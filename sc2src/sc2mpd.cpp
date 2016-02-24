/*
Copyright 2011-14, Linn Products Ltd. All rights reserved.

Unless otherwise stated, all code in this project is licensed under the 2-clause
(Simplified) BSD license.  See BsdLicense.txt for details.

*/
/* Copyright (C) 2014 J.F.Dockes
 *	 This program is free software; you can redistribute it and/or modify
 *	 it under the terms of the GNU General Public License as published by
 *	 the Free Software Foundation; either version 2 of the License, or
 *	 (at your option) any later version.
 *
 *	 This program is distributed in the hope that it will be useful,
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	 GNU General Public License for more details.
 *
 *	 You should have received a copy of the GNU General Public License
 *	 along with this program; if not, write to the
 *	 Free Software Foundation, Inc.,
 *	 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Modified from ohSongcast/Receiver/Receiver.cpp
 */
#include "config.h"

#include <OpenHome/OhNetTypes.h>
#include <OpenHome/Private/Ascii.h>
#include <OpenHome/Private/Thread.h>
#include <OpenHome/Private/OptionParser.h>
#include <OpenHome/Private/Debug.h>
#include <OpenHome/Net/Core/OhNet.h>

#include "Debug.h"
#include "OhmReceiver.h"

#include "workqueue.h"
#include "rcvqueue.h"
#include "log.h"
#include "conftree.h"
#include "chrono.h"

#ifdef WITH_WAVSC2
#include "openaudio.h"
#include "audioreader.h"
#endif

#include <vector>
#include <stdio.h>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;

WorkQueue<AudioMessage*> audioqueue("audioqueue", 4);

#ifdef _WIN32

#pragma warning(disable:4355) // use of 'this' in ctor lists safe in this case

#define CDECL __cdecl

int mygetch()
{
    return (_getch());
}

#else

#define CDECL

#include <termios.h>
#include <unistd.h>

int mygetch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

#endif


using namespace OpenHome;
using namespace OpenHome::Net;
using namespace OpenHome::TestFramework;
using namespace OpenHome::Av;


class OhmReceiverDriver : public IOhmReceiverDriver, public IOhmMsgProcessor {
public:
    OhmReceiverDriver(AudioEater* eater, AudioEater::Context *ctxt);

private:
    // IOhmReceiverDriver
    virtual void Add(OhmMsg& aMsg);
    virtual void Timestamp(OhmMsg& aMsg);
    virtual void Started();
    virtual void Connected();
    virtual void Playing();
    virtual void Disconnected();
    virtual void Stopped();

    // IOhmMsgProcessor
    virtual void Process(OhmMsgAudio& aMsg);
    virtual void Process(OhmMsgTrack& aMsg);
    virtual void Process(OhmMsgMetatext& aMsg);

private:
    // Debug, stats, etc while we get to understand the Songcast streams
    class Observer {
    public:
        TBool iReset;
        TUint iCount;
        TUint iFrame;
        int dumpfd;
        Chrono chron;
        Observer() : iReset(true), iCount(0), iFrame(0), dumpfd(-1) {
#if 0
            dumpfd = 
                open("/y/av/tmp/sc2dump", O_WRONLY|O_CREAT|O_TRUNC, 0666);
            if (dumpfd < 0) {
                LOGERR("OhmReceiverDriver::Open dump file failed\n");
            }
#endif
        }

        void reset() {
            iReset = true;
        }

        void process(OhmMsgAudio& aMsg);
    };
    Observer m_obs;
    AudioEater *m_eater;
};

OhmReceiverDriver::OhmReceiverDriver(AudioEater *eater, 
                                     AudioEater::Context *ctxt)
    : m_eater(eater)
{
    audioqueue.start(1, m_eater->worker, ctxt);
}

void OhmReceiverDriver::Add(OhmMsg& aMsg)
{
    aMsg.Process(*this);
    aMsg.RemoveRef();
}

void OhmReceiverDriver::Timestamp(OhmMsg& /*aMsg*/)
{
}

void OhmReceiverDriver::Started()
{
    LOGDEB("=== STARTED ====\n");
}

void OhmReceiverDriver::Connected()
{
    m_obs.reset();
    printf("CONNECTED\n");
    fflush(stdout);
    LOGDEB("=== CONNECTED ====\n");
}

void OhmReceiverDriver::Playing()
{
    LOGDEB("=== PLAYING ====\n");
}

void OhmReceiverDriver::Disconnected()
{
    LOGDEB("=== DISCONNECTED ====\n");
}

void OhmReceiverDriver::Stopped()
{
    LOGDEB("=== STOPPED ====\n");
}

// Debug and stats only, not needed for main function
void OhmReceiverDriver::Observer::process(OhmMsgAudio& aMsg)
{
    if (++iCount == 400 || aMsg.Halt()) {
        static unsigned long long last_timestamp;
        unsigned long long timestamp = aMsg.MediaTimestamp();
        LOGDEB("OhmRcvDrv::Process:audio: samplerate " << aMsg.SampleRate() <<
               " bitdepth " << aMsg.BitDepth() << " channels " <<
               aMsg.Channels() << " samples " << aMsg.Samples() << 
               " Halted ? " << aMsg.Halt() << endl);

        if (last_timestamp) {
            long long intervalus = 
                ((timestamp - last_timestamp) * 1000000) / (256*48000);
            long long atsus = 
                ((timestamp) * 1000000) / (256*48000);
            long long absus = chron.amicros() - 1430477861905884LL;
            LOGDEB("Computed-uS: " << intervalus  << 
                   " Elapsed-uS: " << chron.urestart() << 
                   " Timestamp-uS: " << atsus <<
                   " Abs-uS: " << absus << 
                   " Diff-mS " << (absus - atsus) / 1000 <<
                   endl);
        }
        last_timestamp = timestamp;

        if (!aMsg.Halt()) {
            unsigned int bytes = 
                aMsg.Samples() * (aMsg.BitDepth() / 8) * aMsg.Channels();

            if (bytes != aMsg.Audio().Bytes()) {
                LOGERR("OhmRcvDrv::Process:audio: computed bytes " << bytes << 
                       " !=  buffer's " << aMsg.Audio().Bytes() << endl);
                bytes = aMsg.Audio().Bytes();
            }
            const unsigned char *icp = 
                (const unsigned char *)aMsg.Audio().Ptr();
            bool silence = true;
            for (unsigned int i = 0; i < bytes; i++) {
                if (icp[i]) {
                    silence = false;
                    break;
                }
            }
            if (silence) {
                LOGDEB("OhmRcvDrv::Process:audio: silence buffer" << endl);
            }
            if (dumpfd >= 0) {
                if (write(dumpfd, icp, bytes) != int(bytes)) {
                    ;
                }
            }
        }

        iCount = 0;
    }

    if (iReset) {
        iFrame = aMsg.Frame();
        iReset = false;
    } else {
        if (aMsg.Frame() != iFrame + 1) {
            LOGINF("Missed frames between " << iFrame << " and " << 
                   aMsg.Frame() << endl);
        }
        iFrame = aMsg.Frame();
    }
}

void copyswap(unsigned char *dest, const unsigned char *src, 
              unsigned int bytes, unsigned int bits)
{
    unsigned char *ocp = dest;
    const unsigned char *icp = src;
    if (bits == 16) {
        swab(src, dest, bytes);
    } else if (bits == 24) {
        while (icp - src <= int(bytes) - 3) {
            *ocp++ = icp[2];
            *ocp++ = icp[1];
            *ocp++ = *icp;
            icp += 3;
        }
    } else if (bits == 32) {
        // Never seen this but whatever...
        while (icp - src <= int(bytes) - 4) {
            *ocp++ = icp[3];
            *ocp++ = icp[2];
            *ocp++ = icp[1];
            *ocp++ = *icp;
            icp += 4;
        }
    }
}

void OhmReceiverDriver::Process(OhmMsgAudio& aMsg)
{
    if (aMsg.Audio().Bytes() == 0) {
        LOGDEB("OhmReceiverDriver::Process: empty message\n");
        return;
    }

    m_obs.process(aMsg);
    if (aMsg.Halt()) {
        return;
    }

    unsigned int bytes = aMsg.Audio().Bytes();
    // We allocate a bit more space to avoir reallocations in the resampler
    unsigned int allocbytes = bytes + 100; 
    char *buf = (char *)malloc(allocbytes);
    if (buf == 0) {
        LOGERR("OhmReceiverDriver::Process: can't allocate " << 
               bytes << " bytes\n");
        return;
    }

    // Songcast data is always msb-first.  Convert to desired order:
    // depends on what downstream wants, and just as well we do it
    // here because we copy the buf anyway.
    bool needswap = false;
    switch (m_eater->input_border) {
    case AudioEater::BO_MSB: 
        break;
    case AudioEater::BO_LSB: 
        needswap = true; 
        break;
    case AudioEater::BO_HOST:
#ifdef WORDS_BIGENDIAN
        needswap = false;
#else
        needswap = true;
#endif
        break;
    }

    if (needswap) {
        copyswap((unsigned char *)buf, aMsg.Audio().Ptr(), bytes, aMsg.BitDepth());
    } else {
        memcpy(buf, aMsg.Audio().Ptr(), bytes);
    }

    AudioMessage *ap = new 
        AudioMessage(aMsg.BitDepth(), aMsg.Channels(), aMsg.Samples(),
                     aMsg.SampleRate(), buf, allocbytes);

    // There is nothing special we can do if put fails: no way to
    // return status. Should we just exit ?
    if (!audioqueue.put(ap, false)) {
        LOGERR("sc2mpd: queue dead: exiting\n");
        exit(1);
    }
}

void OhmReceiverDriver::Process(OhmMsgTrack& aMsg)
{
    Brhz uri(aMsg.Uri());
    Brhz metadata(aMsg.Metadata());
    LOGDEB("OhmRcvDrv::Process:trk: TRACK SEQ " << aMsg.Sequence() <<
           " URI " << uri.CString() <<
           " METADATA " << metadata.CString() << endl);
}

void OhmReceiverDriver::Process(OhmMsgMetatext& aMsg)
{
    Brhz metatext(aMsg.Metatext());
    LOGDEB("OhmRcvDrv::Process:meta: METATEXT SEQUENCE " <<  aMsg.Sequence() <<
           " METATEXT " << metatext.CString() << endl);
}

#ifdef WITH_WAVSC2
static int playWav(const string& wavfile, AudioEater *eater,
                   AudioEater::Context *ctxt)
{
    audioqueue.start(1, eater->worker, ctxt);

    AudioReader *audio = openAudio(wavfile, "", true);

    if (!audio || !audio->open() || audio->bytesPerSample() == 0 ||
        audio->numChannels() == 0) {
        cerr << "Audio file open failed" << endl;
        return 1;
    }
    LOGDEB("sample rate:        " << audio->sampleRate() << endl);
    LOGDEB("sample size:        " << audio->bytesPerSample() << endl);
    LOGDEB("channels:           " << audio->numChannels() << endl);

    size_t packetBytes = 441 * 16 *2;
    while (true) {
        const unsigned char *ibuf = audio->data(packetBytes);
        if (ibuf == 0) {
            return 1;
        }
        // We allocate a bit more space to avoir reallocations in the resampler
        unsigned int allocbytes = packetBytes + 100;
        char *buf = (char *)malloc(allocbytes);
        if (buf == 0) {
            LOGERR("playWav: can't allocate " << allocbytes << " bytes\n");
            return 1;
        }

        // Songcast data is always msb-first.  Convert to desired order:
        // depends on what downstream wants, and just as well we do it
        // here because we copy the buf anyway.
        bool needswap = false;
        switch (eater->input_border) {
        case AudioEater::BO_MSB:
            break;
        case AudioEater::BO_LSB:
            needswap = true;
            break;
        case AudioEater::BO_HOST:
#ifdef WORDS_BIGENDIAN
            needswap = false;
#else
            needswap = true;
#endif
            break;
        }

        int bitDepth = 8 * audio->bytesPerSample();
        if (needswap) {
            copyswap((unsigned char *)buf, ibuf, packetBytes, bitDepth);
        } else {
            memcpy(buf, ibuf, packetBytes);
        }

        // The constructor wants the number of frames as input (frame
        // being all samples at given timepoint, typically 2 for
        // stereo)
        int frames = packetBytes /
            (audio->bytesPerSample() * audio->numChannels());
        AudioMessage *ap = new
            AudioMessage(bitDepth, audio->numChannels(), frames,
                         audio->sampleRate(), buf, allocbytes);

        // There is nothing special we can do if put fails: no way to
        // return status. Should we just exit ?
        if (!audioqueue.put(ap, false)) {
            LOGERR("sc2mpd: queue dead: exiting\n");
            return 1;
        }
    }
    return 0;
}
#endif

int CDECL main(int aArgc, char* aArgv[])
{
    string logfilename;
    int loglevel(Logger::LLINF);

    OptionParser parser;

    OptionUint optionAdapter("-a", "--adapter", 0, 
                             "[adapter] index of network adapter to use");
    parser.AddOption(&optionAdapter);

    OptionUint optionTtl("-t", "--ttl", 1, "[ttl] ttl");
    parser.AddOption(&optionTtl);

    OptionUint optionInteract("-i", "--interact", 0, "[interact] interactive");
    parser.AddOption(&optionInteract);

    OptionString optionUri("-u", "--uri", Brn("mpus://0.0.0.0:0"), 
                           "[uri] uri of the sender");
    parser.AddOption(&optionUri);

    OptionString optionConfig("-c", "--config", Brn("/etc/upmpdcli.conf"), 
                              "[config] upmpdcli configuration file path");
    parser.AddOption(&optionConfig);

    OptionBool optionDevice("-d", "--direct-alsa", 
                            "[stream] Use alsa directly instead of producing "
                            "http stream");
    parser.AddOption(&optionDevice);

#ifdef WITH_WAVSC2
    OptionString optionWav("-w", "--wav somefile.wav", Brn(""), 
                           "Test audio with wav file instead of sender");
    parser.AddOption(&optionWav);
#endif

    if (!parser.Parse(aArgc, aArgv)) {
        cerr << "Bad options, exiting\n";
        return 1;
    }

    string uconfigfile = (const char *)optionConfig.Value().Ptr();

    bool cfspecified = true;
    if (uconfigfile.empty()) {
        cfspecified = false;
        uconfigfile = "/etc/upmpdcli.conf";
    }
    ConfSimple config(uconfigfile.c_str(), 1, true);
    if (!config.ok()) {
        cerr << "Could not open config: " << uconfigfile << endl;
        if (cfspecified)
            return 1;
    }

    config.get("sclogfilename", logfilename);
    string value;
    if (config.get("scloglevel", value))
        loglevel = atoi(value.c_str());
    if (Logger::getTheLog(logfilename) == 0) {
        cerr << "Can't initialize log" << endl;
        return 1;
    }
    Logger::getTheLog("")->setLogLevel(Logger::LogLevel(loglevel));

    AudioEater::Context *ctxt = new AudioEater::Context(&audioqueue);
    ctxt->config = &config;

#ifdef WITH_WAVSC2
    string wavname(Brhz(optionWav.Value()).CString());
    if (!wavname.empty()) {
        string value;
        if (!config.get("sccvttype", value) || 
            value.compare("NONE")) {
            cerr << "Wav input play test. NEEDS sccvttype == NONE\n";
            return 1;
        }
        return playWav(wavname, &alsaAudioEater, ctxt);
    }
#endif

    InitialisationParams* initParams = InitialisationParams::Create();

    Library* lib = new Library(initParams);

    std::vector<NetworkAdapter*>* subnetList = lib->CreateSubnetList();
    TIpAddress subnet = (*subnetList)[optionAdapter.Value()]->Subnet();
    TIpAddress adapter = (*subnetList)[optionAdapter.Value()]->Address();
    Library::DestroySubnetList(subnetList);

    TUint ttl = optionTtl.Value();
    Brhz uri(optionUri.Value());

    LOGINF("scmpdcli: using subnet " << (subnet & 0xff) << "." << 
           ((subnet >> 8) & 0xff) << "." << ((subnet >> 16) & 0xff) << "." <<
           ((subnet >> 24) & 0xff) << endl);

    OhmReceiverDriver* driver = 
        new OhmReceiverDriver(optionDevice.Value() ? 
                              &alsaAudioEater : &httpAudioEater, ctxt);

    OhmReceiver* receiver = new OhmReceiver(lib->Env(), adapter, ttl, *driver);

    CpStack* cpStack = lib->StartCp(subnet);
    cpStack = cpStack; // avoid unused variable warning

    Debug::SetLevel(Debug::kMedia);

    if (optionInteract.Value()) {
        printf("q = quit\n");
        for (;;) {
            int key = mygetch();

            if (key == 'q') {
                printf("QUIT\n");
                break;
            } else if (key == 'p') {
                printf("PLAY %s\n", uri.CString());
                receiver->Play(uri);
            } else if (key == 's') {
                printf("STOP\n");
                receiver->Stop();
            }
        }
    } else {
        receiver->Play(uri);
        for (;;) {
            sleep(1000);
        }
    }

    delete(receiver);

    delete lib;

    if (optionInteract.Value())
        printf("\n");

    return (0);
}

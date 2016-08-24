/*  LilyPad - Pad plugin for PS2 Emulator
 *  Copyright (C) 2002-2014  PCSX2 Dev Team/ChickenLiver
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the
 *  terms of the GNU Lesser General Public License as published by the Free
 *  Software Found- ation, either version 3 of the License, or (at your option)
 *  any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 *  details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with PCSX2.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Global.h"
#include "InputManager.h"

#include "usb.h"
#include "HidDevice.h"


#define VID 0x054c
#define PID 0x0268

 // Unresponsive period required before calling DS3Check().
#define DEVICE_CHECK_DELAY 2000
// Unresponsive period required before calling DS3Enum().  Note that enum is always called on first check.
#define DEVICE_ENUM_DELAY 10000

// Delay between when DS3Check() and DS3Enum() actually do stuff.
#define DOUBLE_CHECK_DELAY 1000
#define DOUBLE_ENUM_DELAY 20000

// Send at least one message every 3 seconds - basically just makes sure the right light(s) are on.
// Not really necessary.
#define UPDATE_INTERVAL 3000




int DualShock3Possible() {
    HANDLE hControlDevice = CreateFileA("\\\\.\\FireShockFilter",
        GENERIC_READ, // Only read access
        0, // FILE_SHARE_READ | FILE_SHARE_WRITE
        nullptr, // no SECURITY_ATTRIBUTES structure
        OPEN_EXISTING, // No special create flags
        0, // No special attributes
        nullptr); // No template file

    // FireShock filter present?
    if (hControlDevice == INVALID_HANDLE_VALUE) {
        return 0;
    }

    CloseHandle(hControlDevice);
    return 1;
}

#include <pshpack1.h>

struct MotorState {
    unsigned char duration;
    unsigned char force;
};

struct LightState {
    // 0xFF makes it stay on.
    unsigned char duration;
    // Have to make one or the other non-zero to turn on light.
    unsigned char dunno[2];
    // 0 is fully lit.
    unsigned char dimness;
    // Have to make non-zero to turn on light.
    unsigned char on;
};

// Data sent to DS3 to set state.
struct DS3Command {
    unsigned char id;
    unsigned char unsure;
    // Small is first, then big.
    MotorState motors[2];
    unsigned char noClue[4];
    // 2 is pad 1 light, 4 is pad 2, 8 is pad 3, 16 is pad 4.  No clue about the others.
    unsigned char lightFlags;
    // Lights are in reverse order.  pad 1 is last.
    LightState lights[4];
    unsigned char dunno[18];
};

#include <poppack.h>

int CharToPressure(unsigned char c) {
    int v = (int)c + ((unsigned int)c >> 7);
    return ((c / 2) * FULLY_DOWN) >> 7;
}

int CharToAxis(unsigned char c) {
    int v = (int)c + ((unsigned int)c >> 7);
    return ((c - 128) * FULLY_DOWN) >> 7;
}

int CharToButton(unsigned char c) {
    int v = (int)c + ((unsigned int)c >> 7);
    return (v * FULLY_DOWN) >> 8;
}

class DualShock3Device : public Device {
    // Cached last vibration values by pad and motor.
    // Need this, as only one value is changed at a time.
    int ps2Vibration[2][4][2];
    int vibration[2];
public:
    int index;
    HANDLE hFile;
    DS3Command sendState;
    unsigned char getState[49];
    OVERLAPPED readop;
    OVERLAPPED writeop;
    int writeCount;
    int lastWrite;

    unsigned int dataLastReceived;

    int writeQueued;
    int writing;

    int StartRead() {
        int res = ReadFile(hFile, &getState, sizeof(getState), 0, &readop);
        return (res || GetLastError() == ERROR_IO_PENDING);
    }

    void QueueWrite() {
        // max of 2 queued writes allowed, one for either motor.
        if (writeQueued < 2) {
            writeQueued++;
            StartWrite();
        }
    }

    int StartWrite() {
        if (!writing && writeQueued) {
            lastWrite = GetTickCount();
            writing++;
            writeQueued--;
            sendState.motors[0].duration = 0x50;
            sendState.motors[1].duration = 0x50;

            int bigForce = vibration[0] * 256 / FULLY_DOWN;
            if (bigForce > 255) bigForce = 255;
            sendState.motors[1].force = (unsigned char)bigForce;
            sendState.motors[0].force = (unsigned char)(vibration[1] >= FULLY_DOWN / 2);
            // Can't seem to have them both non-zero at once.
            if (sendState.motors[writeCount & 1].force) {
                sendState.motors[(writeCount & 1) ^ 1].force = 0;
                sendState.motors[(writeCount & 1) ^ 1].duration = 0;
            }

            writeCount++;
            int res = WriteFile(hFile, &sendState, sizeof(sendState), 0, &writeop);
            return (res || GetLastError() == ERROR_IO_PENDING);
        }
        return 1;
    }

    DualShock3Device(int index, wchar_t *name, wchar_t *path) : Device(DS3, OTHER, name, path, L"DualShock 3") {
        writeCount = 0;
        writing = 0;
        writeQueued = 0;
        memset(&readop, 0, sizeof(readop));
        memset(&writeop, 0, sizeof(writeop));
        memset(&sendState, 0, sizeof(sendState));
        sendState.id = 1;
        int temp = (index & 4);
        sendState.lightFlags = (1 << (temp + 1));
        sendState.lights[3 - temp].duration = 0xFF;
        sendState.lights[3 - temp].dunno[0] = 1;
        sendState.lights[3 - temp].on = 1;
        memset(ps2Vibration, 0, sizeof(ps2Vibration));
        vibration[0] = vibration[1] = 0;
        this->index = index;
        int i;

        // Pressure sensitive buttons
        for (i = 0; i < 12; i++)
        {
            AddPhysicalControl(PRESSURE_BTN, i, 0);
        }

        // Digital buttons
        for (i = 12; i < 16; i++)
        {
            AddPhysicalControl(PSHBTN, i, 0);
        }

        // Axes
        for (i = 16; i < 20; i++)
        {
            AddPhysicalControl(ABSAXIS, i, 0);
        }

        // PS button
        AddPhysicalControl(PSHBTN, 20, 0);

        AddFFAxis(L"Big Motor", 0);
        AddFFAxis(L"Small Motor", 1);

        AddFFEffectType(L"Constant Effect", L"Constant", EFFECT_CONSTANT);

        hFile = INVALID_HANDLE_VALUE;
    }

    wchar_t *GetPhysicalControlName(PhysicalControl *c) {
        const static wchar_t *names[] = {
            L"Up",
            L"Right",
            L"Down",
            L"Left",
            L"Triangle",
            L"Circle",
            L"Cross",
            L"Square",
            L"L1",
            L"L2",
            L"R1",
            L"R2",
            L"L3",
            L"R3",
            L"Select",
            L"Start",
            L"L-Stick X",
            L"L-Stick Y",
            L"R-Stick X",
            L"R-Stick Y",
            L"PS",
        };
        unsigned int i = (unsigned int)(c - physicalControls);
        if (i < sizeof(names) / sizeof(names[0])) {
            return (wchar_t*)names[i];
        }
        return Device::GetPhysicalControlName(c);
    }

    int Activate(InitInfo *initInfo) {
        if (active) Deactivate();
        // Give grace period before get mad.
        lastWrite = dataLastReceived = GetTickCount();
        readop.hEvent = CreateEvent(0, 0, 0, 0);
        writeop.hEvent = CreateEvent(0, 0, 0, 0);
        hFile = CreateFileW(instanceID, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
        if (!readop.hEvent || !writeop.hEvent || hFile == INVALID_HANDLE_VALUE ||
            !StartRead()) {
            Deactivate();
            return 0;
        }
        active = 1;
        AllocState();
        return 1;
    }

    int Update() {
        if (!active) return 0;
        HANDLE h[2] = {
            readop.hEvent,
            writeop.hEvent
        };
        unsigned int time = GetTickCount();
        if (time - lastWrite > UPDATE_INTERVAL) {
            QueueWrite();
        }
        while (1) {
            DWORD res = WaitForMultipleObjects(2, h, 0, 0);
            if (res == WAIT_OBJECT_0) {
                dataLastReceived = time;
                if (!StartRead()) {
                    Deactivate();
                    return 0;
                }

                // Pressure sensitive buttons
                physicalControlState[0] = CharToPressure(getState[10]);
                physicalControlState[1] = CharToPressure(getState[11]);
                physicalControlState[2] = CharToPressure(getState[12]);
                physicalControlState[3] = CharToPressure(getState[13]);
                physicalControlState[4] = CharToPressure(getState[16]);
                physicalControlState[5] = CharToPressure(getState[17]);
                physicalControlState[6] = CharToPressure(getState[18]);
                physicalControlState[7] = CharToPressure(getState[19]);
                physicalControlState[8] = CharToPressure(getState[14]);
                physicalControlState[9] = CharToPressure(getState[8]);
                physicalControlState[10] = CharToPressure(getState[15]);
                physicalControlState[11] = CharToPressure(getState[9]);
                
                // Thumb buttons
                physicalControlState[12] = CharToButton(((getState[6] >> 1) & 1) * 0xFF);
                physicalControlState[13] = CharToButton(((getState[6] >> 2) & 1) * 0xFF);

                // Select & Start
                physicalControlState[14] = CharToButton(((getState[6] >> 0) & 1) * 0xFF);
                physicalControlState[15] = CharToButton(((getState[6] >> 3) & 1) * 0xFF);

                // Thumb axes
                physicalControlState[16] = CharToAxis(getState[1]);
                physicalControlState[17] = CharToAxis(getState[2]);
                physicalControlState[18] = CharToAxis(getState[3]);
                physicalControlState[19] = CharToAxis(getState[4]);

                // PS button
                physicalControlState[20] = CharToButton(((getState[7] >> 0) & 1) * 0xFF);
                
                continue;
            }
            else if (res == WAIT_OBJECT_0 + 1) {
                writing = 0;
                if (!writeQueued && (vibration[0] | vibration[1])) {
                    QueueWrite();
                }
                if (!StartWrite()) {
                    Deactivate();
                    return 0;
                }
            }
            else {
                if (time - dataLastReceived >= DEVICE_CHECK_DELAY) {
                    QueueWrite();
                }
            }
            break;
        }
        return 1;
    }

    void SetEffects(unsigned char port, unsigned int slot, unsigned char motor, unsigned char force) {
        ps2Vibration[port][slot][motor] = force;
        vibration[0] = vibration[1] = 0;
        for (int p = 0; p < 2; p++) {
            for (int s = 0; s < 4; s++) {
                for (int i = 0; i < pads[p][s].numFFBindings; i++) {
                    // Technically should also be a *65535/BASE_SENSITIVITY, but that's close enough to 1 for me.
                    ForceFeedbackBinding *ffb = &pads[p][s].ffBindings[i];
                    vibration[0] += (int)((ffb->axes[0].force * (__int64)ps2Vibration[p][s][ffb->motor]) / 255);
                    vibration[1] += (int)((ffb->axes[1].force * (__int64)ps2Vibration[p][s][ffb->motor]) / 255);
                }
            }
        }
        // Make sure at least 2 writes are queued, to update both motors.
        QueueWrite();
        QueueWrite();
    }

    void SetEffect(ForceFeedbackBinding *binding, unsigned char force) {
        PadBindings pBackup = pads[0][0];
        pads[0][0].ffBindings = binding;
        pads[0][0].numFFBindings = 1;
        SetEffects(0, 0, binding->motor, 255);
        pads[0][0] = pBackup;
    }

    void Deactivate() {
        if (hFile != INVALID_HANDLE_VALUE) {
            CancelIo(hFile);
            CloseHandle(hFile);
            hFile = INVALID_HANDLE_VALUE;
        }
        if (readop.hEvent) {
            CloseHandle(readop.hEvent);
        }
        if (writeop.hEvent) {
            CloseHandle(writeop.hEvent);
        }
        writing = 0;
        writeQueued = 0;
        memset(ps2Vibration, 0, sizeof(ps2Vibration));
        vibration[0] = vibration[1] = 0;

        FreeState();
        active = 0;
    }

    ~DualShock3Device() {
    }
};

void EnumDualShock3s() {
    if (!DualShock3Possible()) return;

    HidDeviceInfo *foundDevs = 0;

    int numDevs = FindHids(&foundDevs, VID, PID);
    if (!numDevs) return;
    int index = 0;
    for (int i = 0; i < numDevs; i++) {
        wchar_t temp[100];
        wsprintfW(temp, L"FS DualShock 3 #%i", index + 1);
        dm->AddDevice(new DualShock3Device(index, temp, foundDevs[i].path));
        index++;

        free(foundDevs[i].path);
    }
    free(foundDevs);
}

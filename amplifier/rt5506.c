/*
 * Copyright (C) 2015, The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include <system/audio.h>

//#define LOG_NDEBUG 0
#define LOG_TAG "rt5506"
#include <cutils/log.h>

#include "rt5506.h"

int rt5506_set_mode(audio_mode_t mode) {
    struct rt5506_comm_data amp_data;
    struct rt5506_config amp_config;
    int headsetohm = HEADSET_OM_UNDER_DETECT;
    int rt5506_fd;
    int ret = -1;

    /* Open the amplifier device */
    if ((rt5506_fd = open(RT5506_DEVICE, O_RDWR)) < 0) {
        ALOGE("error opening amplifier device %s", RT5506_DEVICE);
        return -1;
    }

    /* Get impedance of headset */
    if (ioctl(rt5506_fd, AMP_QUERY_OM, &headsetohm) < 0)
        ALOGE("error querying headset impedance");

    switch(mode) {
        case AUDIO_MODE_NORMAL:
            /* For headsets with a impedance between 128ohm and 1000ohm */
            if (headsetohm >= HEADSET_128OM && headsetohm <= HEADSET_1KOM) {
                ALOGI("Mode: Playback 128");
                amp_config.reg_len = sizeof(rt5506_playback_128_data) / sizeof(rt5506_reg_data);
                memcpy(&amp_config.reg, rt5506_playback_128_data, sizeof(rt5506_playback_128_data));
            } else {
                ALOGI("Mode: Playback");
                amp_config.reg_len = sizeof(rt5506_playback_data) / sizeof(rt5506_reg_data);
                memcpy(&amp_config.reg, rt5506_playback_data, sizeof(rt5506_playback_data));
            }
            amp_data.out_mode = PLAYBACK_MODE_PLAYBACK;
            amp_data.config = amp_config;
            break;
        case AUDIO_MODE_RINGTONE:
            ALOGI("Mode: Ring");
            amp_config.reg_len = sizeof(rt5506_ring_data) / sizeof(rt5506_reg_data);
            memcpy(&amp_config.reg, rt5506_ring_data, sizeof(rt5506_ring_data));
            amp_data.out_mode = PLAYBACK_MODE_RING;
            amp_data.config = amp_config;
            break;
        case AUDIO_MODE_IN_CALL:
        case AUDIO_MODE_IN_COMMUNICATION:
            ALOGI("Mode: Voice");
            amp_config.reg_len = sizeof(rt5506_voice_data) / sizeof(rt5506_reg_data);
            memcpy(&amp_config.reg, rt5506_voice_data, sizeof(rt5506_voice_data));
            amp_data.out_mode = PLAYBACK_MODE_VOICE;
            amp_data.config = amp_config;
            break;
        default:
            ALOGI("Mode: Default");
            amp_config.reg_len = sizeof(rt5506_playback_data) / sizeof(rt5506_reg_data);
            memcpy(&amp_config.reg, rt5506_playback_data, sizeof(rt5506_playback_data));
            amp_data.out_mode = PLAYBACK_MODE_PLAYBACK;
            amp_data.config = amp_config;
            break;
    }

    /* Set the selected config */
    if ((ret = ioctl(rt5506_fd, AMP_SET_CONFIG, &amp_data)) != 0) {
        ALOGE("ioctl %d failed. ret = %d", AMP_SET_CONFIG, ret);
    }

    close(rt5506_fd);

    return ret;
}

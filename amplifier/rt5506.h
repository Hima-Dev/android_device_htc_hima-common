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

#ifndef _RT5506_H_
#define _RT5506_H_

#define RT5506_DEVICE "/dev/rt5506"
#define RT5506_MAX_REG_DATA 15

struct rt5506_reg_data {
    unsigned char addr;
    unsigned char val;
};

struct rt5506_config {
    unsigned int reg_len;
    struct rt5506_reg_data reg[RT5506_MAX_REG_DATA];
};

struct rt5506_comm_data {
    unsigned int out_mode;
    struct rt5506_config config;
};

enum {
    RT5506_INIT = 0,
    RT5506_MUTE,
    RT5506_MAX_FUNC
};

enum RT5506_Mode {
    RT5506_MODE_OFF = RT5506_MAX_FUNC,
    RT5506_MODE_PLAYBACK,
    RT5506_MODE_PLAYBACK8OH,
    RT5506_MODE_PLAYBACK16OH,
    RT5506_MODE_PLAYBACK32OH,
    RT5506_MODE_PLAYBACK64OH,
    RT5506_MODE_PLAYBACK128OH,
    RT5506_MODE_PLAYBACK256OH,
    RT5506_MODE_PLAYBACK500OH,
    RT5506_MODE_PLAYBACK1KOH,
    RT5506_MODE_VOICE,
    RT5506_MODE_TTY,
    RT5506_MODE_FM,
    RT5506_MODE_RING,
    RT5506_MODE_MFG,
    RT5506_MODE_BEATS_8_64,
    RT5506_MODE_BEATS_128_500,
    RT5506_MODE_MONO,
    RT5506_MODE_MONO_BEATS,
    RT5506_MAX_MODE
};

enum HEADSET_OM {
    HEADSET_8OM = 0,
    HEADSET_16OM,
    HEADSET_32OM,
    HEADSET_64OM,
    HEADSET_128OM,
    HEADSET_256OM,
    HEADSET_500OM,
    HEADSET_1KOM,
    HEADSET_MONO,
    HEADSET_OM_UNDER_DETECT,
};

#define RT5506_IOCTL_MAGIC 'g'
#define RT5506_SET_CONFIG   _IOW(RT5506_IOCTL_MAGIC, 0x01,  unsigned)
#define RT5506_READ_CONFIG  _IOW(RT5506_IOCTL_MAGIC, 0x02, unsigned)
#define RT5506_SET_MODE        _IOW(RT5506_IOCTL_MAGIC, 0x03, unsigned)
#define RT5506_SET_PARAM       _IOW(RT5506_IOCTL_MAGIC, 0x04,  unsigned)
#define RT5506_WRITE_REG       _IOW(RT5506_IOCTL_MAGIC, 0x07,  unsigned)
#define RT5506_QUERY_OM       _IOW(RT5506_IOCTL_MAGIC, 0x08,  unsigned)

int rt5506_set_mode(audio_mode_t mode);

#endif

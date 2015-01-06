/*
 * Copyright (C) 2013, The CyanogenMod Project
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

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <cutils/log.h>
#include <sys/ioctl.h>
#include <system/audio.h>

#include "tfa9887.h"

#define LOG_NDEBUG 0
#define LOG_TAG "tfa9887"

/* Module variables */

static bool tfa9887_initialized = false;
static bool tfa9887l_initialized = false;
static bool dsp_enabled = false;
static Tfa9887_Mode_t tfa9887_mode = Tfa9887_Num_Modes;

/* Helper functions */

static int read_file(const char *file_name, uint8_t *buf, int sz) {
    int ret;
    int fd;

    fd = open(file_name, O_RDONLY);
    if (fd < 0) {
        ret = errno;
        ALOGE("%s: unable to open file %s: %d", __func__, file_name, ret);
        return ret;
    }

    ret = read(fd, buf, sz);
    if (ret < 0) {
        ret = errno;
        ALOGE("%s: error reading from file %s: %d", __func__, file_name, ret);
    }

    close(fd);
    return ret;
}

static void bytes2data(const uint8_t bytes[], int num_bytes,
        uint32_t data[]) {
    int i; /* index for data */
    int k; /* index for bytes */
    uint32_t d;
    int num_data = num_bytes/3;

    for (i = 0, k = 0; i < num_data; ++i, k += 3) {
        d = (bytes[k] << 16) | (bytes[k+1] << 8) | (bytes[k+2]);
        /* sign bit was set*/
        if (bytes[k] & 0x80) {
            d = - ((1 << 24) - d);
        }
        data[i] = d;
    }
}

static int tfa9887_read_reg(int fd, uint8_t reg, uint16_t *val) {
    int ret = 0;
    /* kernel uses unsigned int */
    unsigned int reg_val[2];
    uint8_t buf[2];

    reg_val[0] = 2;
    reg_val[1] = (unsigned int) &buf;
    /* unsure why the first byte is skipped */
    buf[0] = 0;
    buf[1] = reg;
    if ((ret = ioctl(fd, TPA9887_WRITE_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_WRITE_CONFIG, ret);
        goto read_reg_err;
    }

    reg_val[0] = 2;
    reg_val[1] = (unsigned int) &buf;
    if ((ret = ioctl(fd, TPA9887_READ_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_READ_CONFIG, ret);
        goto read_reg_err;
    }

    *val = ((buf[0] << 8) | buf[1]);

read_reg_err:
    return ret;
}

static int tfa9887_write_reg(int fd, uint8_t reg, uint16_t val) {
    int ret = 0;
    /* kernel uses unsigned int */
    unsigned int reg_val[2];
    uint8_t buf[4];

    reg_val[0] = 4;
    reg_val[1] = (unsigned int) &buf;
    /* unsure why the first byte is skipped */
    buf[0] = 0;
    buf[1] = reg;
    buf[2] = (0xFF00 & val) >> 8;
    buf[3] = (0x00FF & val);
    if ((ret = ioctl(fd, TPA9887_WRITE_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_WRITE_CONFIG, ret);
        goto write_reg_err;
    }

write_reg_err:
    return ret;
}

static int tfa9887_read(int fd, int addr, uint8_t buf[], int len) {
    int ret = 0;
    uint8_t reg_buf[2];
    unsigned int reg_val[2];
    uint8_t kernel_buf[len];

    reg_val[0] = 2;
    reg_val[1] = (unsigned int) &reg_buf;
    /* unsure why the first byte is skipped */
    reg_buf[0] = 0;
    reg_buf[1] = (0xFF & addr);
    if ((ret = ioctl(fd, TPA9887_WRITE_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_WRITE_CONFIG, ret);
        goto read_err;
    }

    reg_val[0] = len;
    reg_val[1] = (unsigned int) &kernel_buf;
    if ((ret = ioctl(fd, TPA9887_READ_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_READ_CONFIG, ret);
        goto read_err;
    }
    memcpy(buf, kernel_buf, len);

read_err:
    return ret;
}

static int tfa9887_write(int fd, int addr, const uint8_t buf[], int len) {
    int ret = 0;
    /* kernel uses unsigned int */
    unsigned int reg_val[2];
    uint8_t ioctl_buf[len + 2];

    reg_val[0] = len + 2;
    reg_val[1] = (unsigned int) &ioctl_buf;
    /* unsure why the first byte is skipped */
    ioctl_buf[0] = 0;
    ioctl_buf[1] = (0xFF & addr);
    memcpy(ioctl_buf + 2, buf, len);

    if ((ret = ioctl(fd, TPA9887_WRITE_CONFIG, &reg_val)) != 0) {
        ALOGE("ioctl %d failed, ret = %d", TPA9887_WRITE_CONFIG, ret);
        goto write_err;
    }

write_err:
    return ret;
}

static int dsp_read_mem(int fd, uint16_t start_offset,
        int num_words, uint32_t *p_values) {
    uint16_t cf_ctrl; /* the value to sent to the CF_CONTROLS register */
    uint8_t bytes[MAX_I2C_LENGTH];
    int burst_size; /* number of words per burst size */
    int bytes_per_word = 3;
    int num_bytes;
    uint32_t *p;
    int error;

    /* first set DMEM and AIF, leaving other bits intact */
    error = tfa9887_read_reg(fd, TFA9887_CF_CONTROLS, &cf_ctrl);
    if (error != 0) {
        ALOGE("%s: error reading from register %d",
                __func__, TFA9887_CF_CONTROLS);
        goto read_mem_err;
    }
    cf_ctrl &= ~0x000E; /* clear AIF & DMEM */
    cf_ctrl |= (Tfa9887_DMEM_XMEM << 1); /* set DMEM, leave AIF cleared for autoincrement */
    error = tfa9887_write_reg(fd, TFA9887_CF_CONTROLS, cf_ctrl);
    if (error != 0) {
        ALOGE("%s: error writing to register %d",
                __func__, TFA9887_CF_CONTROLS);
        goto read_mem_err;
    }
    error = tfa9887_write_reg(fd, TFA9887_CF_MAD, start_offset);
    if (error != 0) {
        ALOGE("%s: error writing to register %d", __func__, TFA9887_CF_MAD);
        goto read_mem_err;
    }
    num_bytes = num_words * bytes_per_word;
    p = p_values;
    for (; num_bytes > 0; ) {
        burst_size = ROUND_DOWN(MAX_I2C_LENGTH, bytes_per_word);
        if (num_bytes < burst_size) {
            burst_size = num_bytes;
        }
        error = tfa9887_read(fd, TFA9887_CF_MEM, bytes, burst_size);
        if (error != 0) {
            ALOGE("%s: error reading from %d", __func__, TFA9887_CF_MEM);
            goto read_mem_err;
        }
        bytes2data(bytes, burst_size,  p);
        num_bytes -= burst_size;
        p += burst_size / bytes_per_word;
    }

read_mem_err:
    return error;
}

static int processData(int fd, const uint8_t *bytes, int length) {
    int error;
    int size;
    int index = 0;
    int offset;
    uint8_t buffer[MAX_I2C_LENGTH];

    while (index < length) {
        /* extract little endian length */
        size = bytes[index] + bytes[index+1] * 256;
        index += 2;
        if ( (index + size) > length) {
            /* outside the buffer, error in the input data */
            return -1;
        }
        memcpy(buffer, bytes + index, size);
        error = tfa9887_write(fd, buffer[0],
                &buffer[1], size - 1);
        ALOGV("%d %d", buffer[0], size - 1);
        if (error != 0) {
            ALOGE("error");
            break;
        }
        index += size;
    }
    return error;
}

#define PATCH_HEADER_LENGTH 6
static int load_binary_data(int fd, const uint8_t *bytes, int length) {
    int error;
    uint32_t value = 0;
    uint16_t status;
    unsigned short checkrev;
    uint16_t checkaddress;
    uint32_t checkvalue;

    error = tfa9887_read_reg(fd, TFA9887_STATUS, &status);
    if (error != 0) {
        if ((status & 0x0043) != 0x0043) {
            /* one of Vddd, PLL and clocks not ok */
            ALOGE("DSP NOT RUNNING");
            error = -1;
        }
    }

    // check the data lines up
    checkrev = bytes[0];
    checkaddress = (bytes[1] << 8) + bytes[2];
    checkvalue = bytes[5] + (bytes[4] << 8) + (bytes[3] << 16);
    ALOGI("tfa9887 checkrev 0x%x", checkrev); // 0x12
    ALOGI("tfa9887 checkaddress 0x%x", checkaddress); // 0x21b4
    ALOGI("tfa9887 checkvalue 0x%x", checkvalue); // 0x779a

    if (checkaddress != 0xFFFF) {
        error = dsp_read_mem(fd, checkaddress, 1, &value);
        if (error ==  0) {
            if (value != checkvalue) {
                /* DSP subsys not running */
                ALOGE("ERROR: NOT SUPPORTED? checkvalue 0x%x value 0x%x", checkvalue, value);
                //return -1;
            }
        }
    }

    // process buffer
    ALOGI("tfa9887 value %x", value);
    error = processData(fd, bytes + PATCH_HEADER_LENGTH,
                     length - PATCH_HEADER_LENGTH);
    return error;
}

static int dsp_set_param(int fd, uint8_t module_id,
        uint8_t param_id, const uint8_t *data, int num_bytes) {
    int error;
    uint16_t cf_ctrl = 0x0002; /* the value to be sent to the CF_CONTROLS register: cf_req=00000000, cf_int=0, cf_aif=0, cf_dmem=XMEM=01, cf_rst_dsp=0 */
    uint16_t cf_mad = 0x0001; /* memory address to be accessed (0 : Status, 1 : ID, 2 : parameters) */
    uint16_t cf_status; /* the contents of the CF_STATUS register */
    uint8_t mem[3];
    uint8_t id[3];
    uint32_t rpc_status = 0;
    int tries = 0;

    uint8_t buffer[7];
    /* first the data for CF_CONTROLS */
    buffer[0] = ((cf_ctrl >> 8) & 0xFF);
    buffer[1] = (cf_ctrl & 0xFF);
    /* write the contents of CF_MAD which is the subaddress
     * following CF_CONTROLS */
    buffer[2] = ((cf_mad >> 8) & 0xFF);
    buffer[3] = (cf_mad & 0xFF);
    /* write the module and RPC id into CF_MEM, which
     * follows CF_MAD */
    buffer[4] = 0;
    buffer[5] = module_id + 128;
    buffer[6] = param_id;
    error = tfa9887_write(fd, TFA9887_CF_CONTROLS,
                    buffer, sizeof(buffer));

    if (error == 0) {
        int offset = 0;
        int chunk_size = 252;
            /* XMEM word size */
        int remaining_bytes = num_bytes;
        /* due to autoincrement in cf_ctrl, next write will happen at
         * the next address */
        while ((error == 0) && (remaining_bytes > 0)) {
            if (remaining_bytes < chunk_size)
                chunk_size = remaining_bytes;
            /* else chunk_size remains at initialize value above */
            error = tfa9887_write(fd, TFA9887_CF_MEM,
                           data + offset, chunk_size);
            remaining_bytes -= chunk_size;
            offset += chunk_size;
            if (error)
                return -1;
        }
    }

    //error = tfa9887_write_reg(fd, TFA9887_CF_CONTROLS,  0x112);
    //error = tfa9887_write(fd, TFA9887_CF_MEM, data, num_bytes);
    //cf_ctrl = 0x0002;
    error = tfa9887_write_reg(fd, TFA9887_CF_CONTROLS, 0x112);
    if (error == 0) {
        //cf_ctrl |= (1 << 8) | (1 << 4); /* set the cf_req1 and cf_int bit */
        do {
            error = tfa9887_read_reg(fd, TFA9887_CF_STATUS, &cf_status);
            tries++;
            usleep(1000);
            /* don't wait forever, DSP is pretty quick to respond (< 1ms) */
        } while ((error == 0) &&
                ((cf_status & 0x100) == 0) &&
                (tries < 10));

        if (tries >= 10) {
            /* something wrong with communication with DSP */
            ALOGE("%s: Timed out waiting for status  cf_status=0x%04x", __func__, (cf_status & 0x100));
            //error = -1;
        }
    }

    // next
    cf_ctrl = 0x0002;
    cf_mad = 0x0000;
    if (error == 0) {
        error = tfa9887_write_reg(fd, TFA9887_CF_CONTROLS, cf_ctrl);
    }
    if (error == 0) {
        error = tfa9887_write_reg(fd, TFA9887_CF_MAD, cf_mad);
    }
    tries = 0;
    if (error == 0) {
        /* wait for mem to come to stable state */
        do {
            error = tfa9887_read(fd, TFA9887_CF_MEM, mem, 3);
            rpc_status = (int)((mem[0] << 16) | (mem[1] << 8) | mem[2]);
            tries++;
            usleep(1000);
        } while (rpc_status != 0 && tries < 100);
    }
    if (error == 0) {
        if (rpc_status != STATUS_OK) {
            error = rpc_status + 100;
            ALOGE("RPC rpc_status: %d", error);
        }
    }
    return error;
}

/* Module functions */

static int tfa9887_load_dsp(int fd, const char *param_file) {
    int error;
    char *suffix;
    int param_sz;
    int type, module, max_size;

    suffix = strrchr(param_file, '.');
    if (suffix == NULL) {
        ALOGE("%s: Failed to determine parameter file type", __func__);
        error = -EINVAL;
        goto load_dsp_err;
    } else if (strcmp(suffix, ".speaker") == 0) {
        type = PARAM_SET_LSMODEL;
        module = MODULE_SPEAKERBOOST;
        max_size = 423;
    } else if (strcmp(suffix, ".config") == 0) {
        type = PARAM_SET_CONFIG;
        module = MODULE_SPEAKERBOOST;
        max_size = 201;
    } else if (strcmp(suffix, ".preset") == 0) {
        type = PARAM_SET_PRESET;
        module = MODULE_SPEAKERBOOST;
        max_size = 87;
    } else if (strcmp(suffix, ".eq") == 0) {
        type = PARAM_SET_EQ;
        module = MODULE_BIQUADFILTERBANK;
        max_size = 1024;
    } else if (strcmp(suffix, ".drc") == 0) {
        type = PARAM_SET_DRC;
        module = MODULE_SPEAKERBOOST;
        max_size = 381;
    } else {
        ALOGE("%s: Invalid DSP param file %s", __func__, param_file);
        error = -EINVAL;
        goto load_dsp_err;
    }

    uint8_t buf[max_size];
    param_sz = read_file(param_file, buf, max_size);
    if (param_sz < 0) {
        error = param_sz;
        ALOGE("%s: Failed to load file %s: %d", __func__, param_file, error);
        goto load_dsp_err;
    }

    error = dsp_set_param(fd, module, type, buf, param_sz);
    if (error != 0) {
        ALOGE("%s: Failed to set DSP params, error: %d type: %d buf: %d param_sz: %d", __func__, error, type, sizeof(buf), param_sz);
    }

    // set AGC gain insert after speaker write
    //if (strcmp(suffix, ".speaker") == 0) {
    //    uint32_t *data[3];
    //    uint8_t bytes[3];
    //    bytes2data(bytes, 1, data);
    //    dsp_set_param(fd, MODULE_SPEAKERBOOST, SB_PARAM_SET_AGCINS, (uint8_t *)data, 3);
    //}

load_dsp_err:
    return error;
}

static int tfa9887_power(int fd, int on) {
    int error;
    uint16_t value;

    error = tfa9887_read_reg(fd, TFA9887_SYSTEM_CONTROL, &value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_SYSTEM_CONTROL");
        goto power_err;
    }

    // get powerdown bit
    value &= ~(TFA9887_SYSCTRL_POWERDOWN);
    if (!on) {
        value |= TFA9887_SYSCTRL_POWERDOWN;
    }

    error = tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, value);
    if (error != 0) {
        ALOGE("Unable to write TFA9887_SYSTEM_CONTROL");
    }

    if (!on) {
        usleep(1000);
    }

power_err:
    return error;
}

static int tfa9887_set_volume(int fd, float volume) {
    int error;
    uint16_t value;
    uint8_t volume_int;

    if (volume > 0.0) {
        return -1;
    }

    error = tfa9887_read_reg(fd, TFA9887_AUDIO_CONTROL, &value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_AUDIO_CONTROL");
        goto set_vol_err;
    }

    volume = -2.0 * volume;
    volume_int = (((uint8_t) volume) & 0xFF);

    value = ((value & 0x00FF) | (volume_int << 8));
    error = tfa9887_write_reg(fd, TFA9887_AUDIO_CONTROL, value);
    if (error != 0) {
        ALOGE("Unable to write to TFA9887_AUDIO_CONTROL");
        goto set_vol_err;
    }

set_vol_err:
    return error;
}

static int tfa9887_mute(int fd, Tfa9887_Mute_t mute) {
    int error;
    uint16_t aud_value, sys_value;

    error = tfa9887_read_reg(fd, TFA9887_AUDIO_CONTROL, &aud_value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_AUDIO_CONTROL");
        goto mute_err;
    }
    error = tfa9887_read_reg(fd, TFA9887_SYSTEM_CONTROL, &sys_value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_SYSTEM_CONTROL");
        goto mute_err;
    }

    switch (mute) {
        case Tfa9887_Mute_Off:
            /* clear CTRL_MUTE, set ENBL_AMP, mute none */
            aud_value &= ~(TFA9887_AUDIOCTRL_MUTE);
            sys_value |= TFA9887_SYSCTRL_ENBL_AMP;
            break;
        case Tfa9887_Mute_Digital:
            /* set CTRL_MUTE, set ENBL_AMP, mute ctrl */
            aud_value |= TFA9887_AUDIOCTRL_MUTE;
            sys_value |= TFA9887_SYSCTRL_ENBL_AMP;
            break;
        case Tfa9887_Mute_Amplifier:
            /* clear CTRL_MUTE, clear ENBL_AMP, only mute amp */
            aud_value &= ~(TFA9887_AUDIOCTRL_MUTE);
            sys_value &= ~(TFA9887_SYSCTRL_ENBL_AMP);
            break;
        default:
            error = -1;
            ALOGW("Unknown mute type: %d", mute);
            goto mute_err;
    }

    error = tfa9887_write_reg(fd, TFA9887_AUDIO_CONTROL, aud_value);
    if (error != 0) {
        ALOGE("Unable to write TFA9887_AUDIO_CONTROL");
        goto mute_err;
    }
    error = tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, sys_value);
    if (error != 0) {
        ALOGE("Unable to write TFA9887_SYSTEM_CONTROL");
        goto mute_err;
    }

mute_err:
    return error;
}

static int tfa9887_select_input(int fd, int input) {
    int error;
    uint16_t value;

    error = tfa9887_read_reg(fd, TFA9887_I2S_CONTROL, &value);
    if (error != 0) {
        goto select_amp_err;
    }

    // clear 2 bits
    value &= ~(0x3 << TFA9887_I2SCTRL_INPUT_SEL_SHIFT);

    switch (input) {
        case 1:
            value |= 0x40;
            break;
        case 2:
            value |= 0x80;
            break;
        default:
            ALOGW("Invalid input selected: %d",
                    input);
            error = -1;
            goto select_amp_err;
    }
    error = tfa9887_write_reg(fd, TFA9887_I2S_CONTROL, value);

select_amp_err:
    return error;
}

static int tfa9887_select_channel(int fd, int channels) {
    int error;
    uint16_t value;

    error = tfa9887_read_reg(fd, TFA9887_I2S_CONTROL, &value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_I2S_CONTROL");
        goto select_channel_err;
    }

    // clear the 2 bits first
    value &= ~(0x3 << TFA9887_I2SCTRL_CHANSEL_SHIFT);

    switch (channels) {
        case 0:
            value |= 0x8;
            break;
        case 1:
            value |= 0x10;
            break;
        case 2:
            value |= 0x18;
            break;
        default:
            ALOGW("Too many channels requested: %d",
                    channels);
            error = -1;
            goto select_channel_err;
    }
    error = tfa9887_write_reg(fd, TFA9887_I2S_CONTROL, value);
    if (error != 0) {
        ALOGE("Unable to write to TFA9887_I2S_CONTROL");
        goto select_channel_err;
    }

select_channel_err:
    return error;
}

static int tfa9887_set_sample_rate(int fd, int sample_rate) {
    int error;
    uint16_t value;

    error = tfa9887_read_reg(fd, TFA9887_I2S_CONTROL, &value);
    if (error == 0) {
        // clear the 4 bits first
        value &= (~(0xF << TFA9887_I2SCTRL_RATE_SHIFT));
        switch (sample_rate) {
            case 48000:
                value |= TFA9887_I2SCTRL_RATE_48000;
                break;
            case 44100:
                value |= TFA9887_I2SCTRL_RATE_44100;
                break;
            case 32000:
                value |= TFA9887_I2SCTRL_RATE_32000;
                break;
            case 24000:
                value |= TFA9887_I2SCTRL_RATE_24000;
                break;
            case 22050:
                value |= TFA9887_I2SCTRL_RATE_22050;
                break;
            case 16000:
                value |= TFA9887_I2SCTRL_RATE_16000;
                break;
            case 12000:
                value |= TFA9887_I2SCTRL_RATE_12000;
                break;
            case 11025:
                value |= TFA9887_I2SCTRL_RATE_11025;
                break;
            case 8000:
                value |= TFA9887_I2SCTRL_RATE_08000;
                break;
            default:
                ALOGE("Unsupported sample rate %d", sample_rate);
                error = -1;
                return error;
        }
        error = tfa9887_write_reg(fd, TFA9887_I2S_CONTROL, value);
    }

    return error;
}

static int tfa9887_wait_ready(int fd, unsigned int ready_bits,
        unsigned int ready_state) {
    int error;
    uint16_t value;
    int tries;
    bool ready;

    tfa9887_read_reg(fd, TFA9887_STATUS, &value);
    tries = 0;
    do {
        error = tfa9887_read_reg(fd, TFA9887_STATUS, &value);
        ready = (error == 0 &&
                (value & ready_bits) == ready_state);
        ALOGD("Waiting for 0x%04x, current state: 0x%04x",
                ready_state, value & ready_bits);
        tries++;
        usleep(1000);
    } while (!ready && tries < 10);

    if (tries >= 10) {
        ALOGE("Timed out waiting for tfa9887 to become ready");
        error = -1;
    }

    return error;
}

static int tfa9887_set_configured(int fd) {
    int error;
    uint16_t value;

    error = tfa9887_read_reg(fd, TFA9887_SYSTEM_CONTROL, &value);
    if (error != 0) {
        ALOGE("Unable to read from TFA9887_SYSTEM_CONTROL");
        goto set_conf_err;
    }
    value |= TFA9887_SYSCTRL_CONFIGURED;
    tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, value);
    if (error != 0) {
        ALOGE("Unable to write TFA9887_SYSTEM_CONTROL");
    }

set_conf_err:
    return error;
}

static int tfa9887_startup(int fd) {
    int error;
    uint16_t value = 0;
    uint16_t value2 = 0x12;

    tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, 0x2);

    tfa9887_read_reg(fd, 0x8, &value2);
    if ( value2 & 0x400 ) {
        tfa9887_write_reg(fd, 0x8, value2 & 0xFBFF);
        tfa9887_read_reg(fd, 0x8, &value2);
        tfa9887_write_reg(fd, 0x8, value2);
    }

    error = tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, 0x2);
    if (0 == error) {
        error = tfa9887_read_reg(fd, TFA9887_SYSTEM_CONTROL, &value);
    }
    if (0 == error) {
        /* DSP must be in control of the amplifier to avoid plops */
        value |= TFA9887_SYSCTRL_SEL_ENBL_AMP;
        error = tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, value);
    }

    /* some other registers must be set for optimal amplifier behaviour */
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_BAT_PROT, 0x13AB);
    }
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_AUDIO_CONTROL, 0x1F);
    }
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_SPKR_CALIBRATION, 0x3C4E);
    }
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_SYSTEM_CONTROL, 0x24D);
    }
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_PWM_CONTROL, 0x308);
    }
    if (0 == error) {
        error = tfa9887_write_reg(fd, TFA9887_CURRENTSENSE4, 0xE82);
    }

    return error;
}

static int is_Dsp_Calibrated(int fd) {
  uint16_t value;

  tfa9887_read_reg(fd, TFA9887_MTP, &value);
  return (value >> 1) & 1;
}

//static void getStatus() {
//    uint8_t dummy_data[16];
//    dummy_data = 0xFFFFFF;
//    load_binary_data(fd, dummy_data, 0x10);
//}
static int tfa9887_init(int fd, int sample_rate,
        bool is_right) {
    int error;
    const char *patch_file, *speaker_file;
    uint8_t patch_data[MAX_PATCH_SIZE];
    int patch_sz;
    int channel;
    uint16_t status, acs_status, mtp, mtp2, syctrl, otcStatus;
    int timeout = 0;
    uint32_t dummy_data[4];
    unsigned int pll_lock_bits = (TFA9887_STATUS_CLKS | TFA9887_STATUS_PLLS);
    int v2;

    ALOGE("tfa9887_init: start");

    if (is_right) {
        channel = 1;
        patch_file = MASTER_PATCH;
        speaker_file = SPKR_R;
    } else {
        channel = 0;
        patch_file = MASTER_PATCH;
        speaker_file = SPKR_L;
    }

    /* must wait until chip is ready otherwise no init happens */
   // error = tfa9887_wait_ready(fd, TFA9887_STATUS_MTPB, 0);
   // if (error != 0) {
   //     ALOGE("tfa9887 MTP still busy");
   //     goto priv_init_err;
   // }

    /* do cold boot init */
    error = tfa9887_startup(fd);
    if (error != 0) {
        ALOGE("Unable to cold boot");
        goto priv_init_err;
    }
    error = tfa9887_set_sample_rate(fd, sample_rate);
    if (error != 0) {
        ALOGE("Unable to set sample rate");
        goto priv_init_err;
    }
    error = tfa9887_select_channel(fd, channel);
    if (error != 0) {
        ALOGE("Unable to select channel");
        goto priv_init_err;
    }
    error = tfa9887_select_input(fd, 2);
    if (error != 0) {
        ALOGE("Unable to select input");
        goto priv_init_err;
    }
 //   error = tfa9887_set_volume(fd, 0.0);
 //   if (error != 0) {
 //       ALOGE("Unable to set volume");
 //       goto priv_init_err;
 //   }
    error = tfa9887_power(fd, true);
    if (error != 0) {
        ALOGE("Unable to power up");
        goto priv_init_err;
    }
    usleep(5000);

    /* wait for ready */
    //error = tfa9887_wait_ready(fd, 2, 0);
    //if (error != 0) {
    //    ALOGE("Failed to lock PLLs");
    //    goto priv_init_err;
    //}


    //void *buffer;
    error = tfa9887_read_reg(fd, 0x00, &acs_status);
    if (error != 0) {
        ALOGE("Failed to lock PLLs");
        goto priv_init_err;
    }

    ALOGE("coldstart status1:0x%x", acs_status);

    if ((acs_status & 2)) {
        ALOGE("tfa9887_read_reg winnering.");
    } else {

    timeout = 0;
    while ((acs_status & 0x2) == 0 && timeout != 10) {
        /* not ok yet */
        error = tfa9887_read_reg(fd, 0x00, &acs_status);
        usleep(1000);
        if (error != 0) {
            ALOGE("tfa9887_read_reg failed.");
            return -1;
        }
        timeout++;
        if (timeout > 10) {
            ALOGE("timeout status:0x%x", (acs_status & 0x2));
            //return -1;
        }
    }
    }
    dummy_data[0] = 0xffffff;
    dummy_data[1] = 0x80000;
    dummy_data[2] = 0x81070070;
    dummy_data[3] = 0x1000000;

    timeout = 0;
    load_binary_data(fd, (uint8_t *) &dummy_data, 0x10);
    tfa9887_read_reg(fd, 0x00, &status);
    tfa9887_read_reg(fd, 0x9, &syctrl);
    tfa9887_read_reg(fd, 0x80, &mtp);
    ALOGD("status = 0x%x, mtp:0x%x, systcl:0x%x", status, mtp, syctrl);

    error = tfa9887_read_reg(fd, 0x0, &acs_status);
    if (error != 0) {
        ALOGE("tfa9887_read_reg failed.");
        return -1;
    }
/*
    while ((acs_status & TFA98XX_STATUSREG_ACS_MSK) == 0) {
        // not ok yet 
        error = tfa9887_read_reg(fd, (acs_status & TFA98XX_STATUSREG_ACS_MSK), &acs_status);
        if (error != 0) {
            ALOGE("tfa9887_read_reg failed.");
            return -1;
        }
        load_binary_data(fd, (uint8_t *) &dummy_data, 0x10);
        tfa9887_read_reg(fd, 0x00, &status);
        tfa9887_read_reg(fd, 0x9, &syctrl);
        tfa9887_read_reg(fd, 0x80, &mtp);
        ALOGD("status = 0x%x, mtp:0x%x, systcl:0x%x", status, mtp, syctrl);
        usleep(5000);

        timeout++;
        if (timeout > 10) {
            ALOGE("timeout status:0x%x", (acs_status & 0x800));
            return -1;
        }
    }
*/
    ALOGE("coldstart status2:0x%x", acs_status);

    /* load firmware */
    patch_sz = read_file(patch_file, patch_data, MAX_PATCH_SIZE);
    if (patch_sz < 0) {
        ALOGE("Unable to read patch file");
        goto priv_init_err;
    }

    error = load_binary_data(fd, patch_data, patch_sz);
    if (error != 0) {
        ALOGE("Unable to load patch data");
        goto priv_init_err;
    }

    tfa9887_read_reg(fd, 0x80, &mtp2);
    if ( mtp2 & 1) {
        tfa9887_write_reg(fd, 0xB, 0x5A);
        tfa9887_write_reg(fd, 0x80, 1);
        tfa9887_write_reg(fd, 0x62, 0x800);
        tfa9887_write_reg(fd, 0x70, 1);
    }

    timeout = 0;
    do {
        usleep(10000);
        error = tfa9887_read_reg(fd, 0x00, &otcStatus);
        if (error != 0) {
            goto priv_init_err;
        }
        timeout++;
        if (timeout > 50) {
            goto priv_init_err;
        }
    } while ((status & TFA98XX_STATUSREG_MTPB_MSK)
        == TFA98XX_STATUSREG_MTPB_MSK);

    if (is_Dsp_Calibrated(fd)) {
        ALOGE("DSP already calibrated. Calibration results loaded from MTP.");
    } else {
        ALOGE("DSP not yet calibrated. Calibration will start.");
        error = tfa9887_load_dsp(fd, speaker_file);
        if (error != 0) {
            ALOGE("Unable to load speaker data");
            goto priv_init_err;
        }
        //tfa9887_mute(fd, Tfa9887_Mute_Digital);
        // TODO manual calibration
    }
    ALOGE("tfa9887_init: end");
priv_init_err:
    return error;
}

static int tfa9887_set_dsp_mode(int fd, Tfa9887_Mode_t mode, bool is_right) {
    int error;
    const struct mode_config *config;

    if (is_right) {
        config = Tfa9887_Right_Mode_Configs;
        ALOGD("Setting right mode to %d", mode);
    } else {
        config = Tfa9887_Left_Mode_Configs;
        ALOGD("Setting left mode to %d", mode);
    }

    error = tfa9887_load_dsp(fd, config[mode].config);
    if (error != 0) {
        ALOGE("Unable to load config data");
        goto set_dsp_err;
    }
    error = tfa9887_load_dsp(fd, config[mode].preset);
    if (error != 0) {
        ALOGE("Unable to load preset data");
        goto set_dsp_err;
    }
    error = tfa9887_load_dsp(fd, config[mode].eq);
    if (error != 0) {
        ALOGE("Unable to load EQ data");
        goto set_dsp_err;
    }

    error = tfa9887_load_dsp(fd, config[mode].drc);
    if (error != 0) {
        ALOGE("Unable to load DRC data");
        goto set_dsp_err;
    }

    error = tfa9887_set_configured(fd);
    if (error != 0) {
        ALOGE("Unable to set configured");
        goto set_dsp_err;
    }

    /* wait for ready */
    error = tfa9887_wait_ready(fd, TFA9887_STATUS_MTPB, 0);
    if (error != 0) {
        ALOGE("Failed to become ready");
        goto set_dsp_err;
    }

set_dsp_err:
    return error;
}

static Tfa9887_Mode_t tfa9887_get_mode(audio_mode_t mode) {
    switch (mode) {
        case AUDIO_MODE_RINGTONE:
            return Tfa9887_Mode_Ring;
        case AUDIO_MODE_IN_CALL:
        case AUDIO_MODE_IN_COMMUNICATION:
            return Tfa9887_Mode_Voice;
        case AUDIO_MODE_NORMAL:
        default:
            return Tfa9887_Mode_Playback;
    }
}

/* Public functions */

int tfa9887_set_mode(audio_mode_t mode) {
    unsigned int reg_value[2];
    int tfa9887_fd;
    int tfa9887l_fd;
    int ret = -1;
    Tfa9887_Mode_t dsp_mode;

    dsp_mode = tfa9887_get_mode(mode);

    /* Open the amplifier devices */
    if ((tfa9887_fd = open(TFA9887_DEVICE, O_RDWR)) < 0) {
        ALOGE("error opening amplifier device %s", TFA9887_DEVICE);
        return -1;
    }
    if ((tfa9887l_fd = open(TFA9887L_DEVICE, O_RDWR)) < 0) {
        ALOGE("error opening amplifier device %s", TFA9887L_DEVICE);
        return -1;
    }

    /* Lock TFA9887 kernel driver */
    reg_value[0] = 1;
    reg_value[1] = 1;
    if ((ret = ioctl(tfa9887_fd, TPA9887_KERNEL_LOCK, &reg_value)) != 0) {
        ALOGE("ioctl %d failed. ret = %d", TPA9887_KERNEL_LOCK, ret);
        goto set_mode_unlock;
    }
    if ((ret = ioctl(tfa9887l_fd, TPA9887_KERNEL_LOCK, &reg_value)) != 0) {
        ALOGE("ioctl %d failed. ret = %d", TPA9887_KERNEL_LOCK, ret);
        goto set_mode_unlock;
    }

    if (tfa9887_initialized && tfa9887l_initialized &&
            dsp_mode == tfa9887_mode) {
        ALOGI("No mode change needed, already mode %d", dsp_mode);
        goto set_mode_unlock;
    }

    /* Mute to avoid pop */
    //ret = tfa9887_mute(tfa9887_fd, Tfa9887_Mute_Digital);
    //ret = tfa9887_mute(tfa9887l_fd, Tfa9887_Mute_Digital);

    /* Enable DSP if necessary */
    reg_value[0] = 1; // length (unused)
    reg_value[1] = 0; // enable
    if (!tfa9887_initialized) {
        ALOGE("enable DSP for tfa9887R");
        if ((ret = ioctl(tfa9887_fd, TPA9887_ENABLE_DSP, &reg_value)) != 0) {
            ALOGE("ioctl %d failed. ret = %d", TPA9887_ENABLE_DSP, ret);
            goto set_mode_unmute;
        }
    }

    /* Initialize if necessary */
    if (!tfa9887_initialized) {
        ALOGE("initialize tfa9887R");
        ret = tfa9887_init(tfa9887_fd, TFA9887_DEFAULT_RATE, true);
        if (ret != 0) {
            ALOGE("Failed to initialize tfa9887R, DSP not enabled");
            goto set_mode_unmute;
        }
        tfa9887_initialized = true;
    }

    /* Enable DSP if necessary */
    reg_value[0] = 1; // length (unused)
    reg_value[1] = 1; // enable
    if (tfa9887_initialized) {
        ALOGE("enable DSP for tfa9887R");
        if ((ret = ioctl(tfa9887_fd, TPA9887_ENABLE_DSP, &reg_value)) != 0) {
            ALOGE("ioctl %d failed. ret = %d", TPA9887_ENABLE_DSP, ret);
            goto set_mode_unmute;
        }
    }

    reg_value[0] = 1; // len (unused)
    reg_value[1] = 0; // enable
    if (!tfa9887l_initialized) {
        ALOGE("enable DSP for tfa9887L");
        if ((ret = ioctl(tfa9887l_fd, TPA9887_ENABLE_DSP, &reg_value)) != 0) {
            ALOGE("ioctl %d failed. ret = %d", TPA9887_ENABLE_DSP, ret);
            goto set_mode_unmute;
        }
    }

    if (!tfa9887l_initialized) {
        ALOGE("initialize tfa9887L");
        ret = tfa9887_init(tfa9887l_fd, TFA9887_DEFAULT_RATE, false);
        if (ret != 0) {
            ALOGE("Failed to initialize tfa9887L, DSP not enabled");
            goto set_mode_unmute;
        }
        tfa9887l_initialized = true;
    }

    reg_value[0] = 1; // len (unused)
    reg_value[1] = 1; // enable
    if (tfa9887l_initialized) {
        ALOGE("enable DSP for tfa9887L");
        if ((ret = ioctl(tfa9887l_fd, TPA9887_ENABLE_DSP, &reg_value)) != 0) {
            ALOGE("ioctl %d failed. ret = %d", TPA9887_ENABLE_DSP, ret);
            goto set_mode_unmute;
        }
    }

    /* Set DSP mode */
    ret = tfa9887_set_dsp_mode(tfa9887_fd, dsp_mode, true);
    if (ret != 0) {
        ALOGE("Failed to set TFA9887R DSP mode: %d", ret);
        goto set_mode_unmute;
    }
    ret = tfa9887_set_dsp_mode(tfa9887l_fd, dsp_mode, false);
    if (ret != 0) {
        ALOGE("Failed to set TFA9887L DSP mode: %d", ret);
        goto set_mode_unmute;
    }

    ALOGI("Set DSP mode to %d", dsp_mode);
    tfa9887_mode = dsp_mode;

set_mode_unmute:
    ret = tfa9887_mute(tfa9887_fd, Tfa9887_Mute_Off);
    ret = tfa9887_mute(tfa9887l_fd, Tfa9887_Mute_Off);

set_mode_unlock:
    /* Unlock TFA9887 kernel driver */
    reg_value[0] = 1;
    reg_value[1] = 0;
    if ((ret = ioctl(tfa9887_fd, TPA9887_KERNEL_LOCK, &reg_value)) != 0) {
        ALOGE("ioctl %d failed. ret = %d", TPA9887_KERNEL_LOCK, ret);
    }
    if ((ret = ioctl(tfa9887l_fd, TPA9887_KERNEL_LOCK, &reg_value)) != 0) {
        ALOGE("ioctl %d failed. ret = %d", TPA9887_KERNEL_LOCK, ret);
    }

    close(tfa9887_fd);
    close(tfa9887l_fd);

    return ret;
}

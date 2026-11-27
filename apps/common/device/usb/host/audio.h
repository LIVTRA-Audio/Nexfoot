/* Modified for Nexfoot; upstream notices are retained. */
#ifndef  __AUDIO_H__
#define  __AUDIO_H__

#include "app_config.h"
#include "system/task.h"
#include "device/device.h"
#include "usb_bulk_transfer.h"
#include "usb/host/usb_host.h"
#include "usb/device/uac_audio.h"


#define HEADPHONE_SUPPORTED                0x01
#define MICROPHONE_SUPPORTED               0x02
#define HEADSET_SUPPORTED                  0x03

struct audio_streaming_t {
    u8  bNumEndpoints;
#if !CONFIG_MD400_USB_HOST_MIDI_ONLY
    u8  bFormatType;		/** FORMAT_TYPE_1 */
    u8  bNrChannels;		/** physical channels in the stream */
    u8  bSubframeSize;
    u8  bBitResolution;
    u8  bSamFreqType;
    u32 tSamFreq[8];
#endif
    u8  host_ep; //主机传输端点
    u8  ep; //从机端点(由描述符中指定)
    u8  ep_Interval;
    u16 ep_max_packet_size;
};

struct audio_device_t {
    u8 interface_num; //接口号
    u8 subclass;
    u8 support;
    void *parent;
#if CONFIG_MD400_USB_HOST_MIDI_ONLY
    /* MIDI has one bulk IN and one bulk OUT record. Keeping the audio
     * alternate-setting/sample-rate tables here wasted 2976 bytes on AC632. */
    struct audio_streaming_t as[2];
#else
    struct audio_streaming_t as[8];
#endif
};

int usb_audio_parser(struct usb_host_device *host_dev, u8 interface_num, const u8 *pBuf);

// API
int usb_audio_play_put_buf(void *ptr, u32 len);
int usb_audio_record_get_buf(void *ptr, u32 len);

//headphone api
void set_usb_audio_play_volume(u16 vol);
void usb_audio_start_play(const usb_dev usb_id, u8 channel, u8 bit_reso, u32 sample_rate); //指定播放数据的声道数,位数,采样率
void usb_audio_stop_play(const usb_dev usb_id);
void usb_audio_pause_play(void);
void usb_audio_resume_play(void);

//microphone api
void set_usb_audio_record_volume(u16 vol);
void usb_audio_start_record(const usb_dev usb_id, u8 bit_reso, u32 sample_rate); //指定录制数据的位数,采样率
void usb_audio_stop_record(const usb_dev usb_id);
void usb_audio_pause_record(void);
void usb_audio_resume_record(void);

// [2025-12-09] MIDI Host API: bulk send/receive on USB Host
int usb_midi_host_send(const usb_dev usb_id, const u8 *data, u32 len);
int usb_midi_host_receive(const usb_dev usb_id, u8 *buf, u32 len);
#endif

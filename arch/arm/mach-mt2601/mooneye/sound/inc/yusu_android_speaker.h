
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <mach/mt_typedefs.h>

#ifndef _YUSU_ANDROID_SPEAKER_H_
#define _YUSU_ANDROID_SPEAKER_H_

enum EAMP_CONTROL_SUBCOMMAND
 {
	 EAMP_SPEAKER_CLOSE =0,
	 EAMP_SPEAKER_OPEN,
	 EAMP_HEADPHONE_OPEN,
	 EAMP_HEADPHONE_CLOSE,
	 EAMP_EARPIECE_OPEN,
	 EAMP_EARPIECE_CLOSE,
	 EAMP_GETREGISTER_VALUE,
	 EAMP_SETREGISTER_VALUE,
	 EAMP_SETAMP_GAIN,
	 EAMP_GETAMP_GAIN,
	 EAMP_GET_CTRP_NUM ,
	 EAMP_GET_CTRP_BITS,
	 EAMP_GET_CTRP_TABLE,
	 EAMP_SETMODE,
 };

enum AUDIO_AMP_CONTROL_COMMAND{
    AUD_AMP_GET_CTRP_NUM ,
    AUD_AMP_GET_CTRP_BITS,
    AUD_AMP_GET_CTRP_TABLE,
    AUD_AMP_GET_REGISTER,
    AUD_AMP_SET_REGISTER,
    AUD_AMP_SET_AMPGAIN,  // gain is use for low 24bits as external amp , device should base on control point set to AMPLL_CON0_REG
    AUD_AMP_GET_AMPGAIN,
    AUD_AMP_SET_MODE,
    NUM_AUD_AMP_COMMAND
};

typedef struct {
	unsigned long int	command;
	unsigned long int 	param1;
	unsigned long int 	param2;
}AMP_Control;

enum SPEAKER_CHANNEL
{
      Channel_None = 0 ,
      Channel_Right,
      Channel_Left,
      Channel_Stereo
};

bool Speaker_Init(void);
bool Speaker_DeInit(void);
bool Speaker_Register(void);
int  ExternalAmp(void);

void Sound_Speaker_Turnon(int channel);
void Sound_Speaker_Turnoff(int channel);
void Sound_Speaker_SetVolLevel(int level);

void Sound_Headset_Turnon(void);
void Sound_Headset_Turnoff(void);

//now for  kernal use
void AudioAMPDevice_Suspend(void);
void AudioAMPDevice_Resume(void);
// used for AEE beep sound
void AudioAMPDevice_SpeakerLouderOpen(void); //some times kernal need to force  speaker for notification
void AudioAMPDevice_SpeakerLouderClose(void);
void AudioAMPDevice_mute(void);

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count);

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size);


#endif



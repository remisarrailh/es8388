/*
	AC101 Codec driver library example.
	Uses the ESP32-A1S module with integrated AC101 codec, mounted on the ESP32 Audio Kit board:
	https://wiki.ai-thinker.com/esp32-audio-kit

	Required library: ESP8266Audio https://github.com/earlephilhower/ESP8266Audio

	Copyright (C) 2019, Ivo Pullens, Emmission

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorRTTTL.h"
#include "AudioOutputI2S.h"
#include "ES8388.h"
#include <driver/i2s.h>
const char song[] PROGMEM =
  "Batman:d=8,o=5,b=180:d,d,c#,c#,c,c,c#,c#,d,d,c#,c#,c,c,c#,c#,d,d#,c,c#,c,c,c#,c#,f,p,4f";
// Plenty more at: http://mines.lumpylumpy.com/Electronics/Computers/Software/Cpp/MFC/RingTones.RTTTL

AudioGeneratorRTTTL *rtttl;
AudioFileSourcePROGMEM *file;
AudioOutputI2S *out;

#define IIS_SCLK                    5
#define IIS_LCLK                    25
#define IIS_DSIN                    26

#define IIC_CLK                     23
#define IIC_DATA                    18

#define GPIO_PA_EN                  GPIO_NUM_21
#define GPIO_SEL_PA_EN              GPIO_SEL_21

#define PIN_PLAY                    (36)      // KEY 4
#define PIN_VOL_UP                  (13)      // KEY 5
#define PIN_VOL_DOWN                (19)       // KEY 6

#define SPEAKER_I2S_NUMBER I2S_NUM_0

static ES8388 es;

static uint8_t volume = 100;
const uint8_t volume_step = 2;
unsigned long debounce = 0;

void setup()
{
  Serial.begin(115200);

  Serial.printf("Connect to ES8388 codec... ");
  
    while (not es.begin(IIC_DATA, IIC_CLK))
    {
  	Serial.printf("Failed!\n");
  	delay(1000);
    }
    Serial.printf("OK\n");
  
  es.volume(ES8388::ES_MAIN, volume);
  es.volume(ES8388::ES_OUT1, volume);
  es.volume(ES8388::ES_OUT2, volume);
  es.mute(ES8388::ES_OUT1, false);
  es.mute(ES8388::ES_OUT2, false);
  es.mute(ES8388::ES_MAIN, false);
  //es.SetVolumeHeadphone(volume);
  //  ac.DumpRegisters();

  // Enable amplifier
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  // Configure keys on ESP32 Audio Kit board

  // Create audio source from progmem, enable I2S output,
  // configure I2S pins to matchn the board and create ringtone generator.
  file = new AudioFileSourcePROGMEM( song, strlen_P(song) );
  out = new AudioOutputI2S();
  out->SetPinout(IIS_SCLK /*bclkPin*/, IIS_LCLK /*wclkPin*/, IIS_DSIN /*doutPin*/);
  rtttl = new AudioGeneratorRTTTL();

  Serial.printf("Use KEY4 to play, KEY5/KEY6 for volume Up/Down\n");
  //file->seek(0, SEEK_SET);
  rtttl->begin(file, out);
}

void loop()
{
  if (rtttl->isRunning())
  {
    Serial.println("Running");
    if (!rtttl->loop())
    {
      Serial.println("END");
      rtttl->stop();
      // Last note seems to loop after stop.
      // To silence also set volume to 0.
      es.volume(ES8388::ES_MAIN, 0);
      //ac.SetVolumeHeadphone(0);
    }
  }
}
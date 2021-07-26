import gtts
from playsound import playsound
import os
import pyttsx3

path = "./tts_demofile.txt"
# f = open(path, "r")
# print(f.readline())
# line = f.readline()
# print(line)

# while True:
#     try:
#         if os.stat(path).st_size != 0:
#             line = f.readline()
#             tts = gtts.gTTS(line)
#             tts.save("./hola.mp3")
#             playsound("./hola.mp3")
#         else:
#             pass
#     except:
#         print("An exception occurred")
#         break


# initialize Text-to-speech engine
engine = pyttsx3.init()
f = open(path, "r")
# convert this text to speech

while True:
    text = f.readline()
    engine.say(text)
    # play the speech
    engine.runAndWait()


# while True:
#     if os.stat(path).st_size != 0:
#         f = open(path, "r")
#         line = f.readline()
#         tts = gtts.gTTS(line)
#         tts.save("./hola.mp3")
#         playsound("./hola.mp3")
#     else:
#         pass



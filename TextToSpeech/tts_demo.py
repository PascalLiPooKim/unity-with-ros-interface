import gtts
from playsound import playsound
import os
import pyttsx3
from multiprocessing import Process
import time

path = "./TTS-Texts/DistanceToGoalTTS.txt"
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
# engine = pyttsx3.init()
# f = open(path, "r")
# # convert this text to speech

# while True:
#     text = f.readline()
#     engine.say(text)
#     # play the speech
#     engine.runAndWait()


# while True:
#     if os.stat(path).st_size != 0:
#         f = open(path, "r")
#         line = f.readline()
#         tts = gtts.gTTS(line)
#         tts.save("./hola.mp3")
#         playsound("./hola.mp3")
#     else:
#         pass

def text_to_speech():
    engine = pyttsx3.init()
    f = open(path, "r")
    # count = 0
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate+20)
    while True:
        # if count < 50:
        text = f.readline()
        engine.say(text)
        # play the speech
        engine.runAndWait()
        time.sleep(1)
        # print(count)
        # count += 1
        
        # if engine.isBusy():
        #     count = 0
        # engine.runAndWait()
    # else:
    #     break
# initialize Text-to-speech engine

# convert this text to speech
# count = 0
if __name__ == '__main__':
    time.sleep(10)
    action_process = Process(target=text_to_speech)
 
    # We start the process and we block for 5 seconds.
    action_process.start()
    action_process.join(timeout=600)
 
    # We terminate the process.
    action_process.terminate()
    print("Text-to-Speech Function Timeout")

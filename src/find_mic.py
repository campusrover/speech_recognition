import speech_recognition as sr 
from ctypes import *


ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    # print('messages are yummy')
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)

for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
    print(i,": ",microphone_name)

r = sr.Recognizer()

with sr.Microphone(device_index=10) as source:  
    print("Please wait. Calibrating microphone...")  
    # listen for 5 seconds and create the ambient noise energy level  
    r.adjust_for_ambient_noise(source, duration=5)  
    print("Say something!")
    audio = r.listen(source)

result = r.recognize_google(audio)
print(result)

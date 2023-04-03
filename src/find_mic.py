import speech_recognition as sr 


# for i, microphone_name in enumerate(sr.Microphone.list_microphone_names()):
#     print(i,": ",microphone_name)

r = sr.Recognizer()

with sr.Microphone(device_index=16) as source:  
    print("Please wait. Calibrating microphone...")  
    # listen for 5 seconds and create the ambient noise energy level  
    r.adjust_for_ambient_noise(source, duration=5)  
    print("Say something!")
    audio = r.listen(source)

result = r.recognize_google(audio)
print(result)

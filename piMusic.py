import pygame
import time

def pygameInit():
    pygame.init()
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=4096)

def playAudio(MP3_Location, start_time, volume):
    audio = "%s" % MP3_Location
    pygame.mixer.music.load(audio)
    pygame.mixer.music.play()
    pygame.mixer.music.set_pos(start_time)
    pygame.mixer.music.set_volume(volume)
    
def busy():
    while pygame.mixer.music.get_busy():
        time.sleep(0.01)

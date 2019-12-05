from gtts import gTTS


if __name__ == '__main__':
    file_contents = {
        '0.mp3': 'There are no shapes',
        '1.mp3': 'I see 1 shape',
        '2.mp3': 'I see 2 shapes',
        '3.mp3': 'I see 3 shapes',
        'match.mp3': 'I found the matching shape',
        'parked.mp3': 'Parked at the square containing the matching shape',
        'artag.mp3': 'Found marker',
        'unmarked.mp3': 'Found unmarked spot',
        'pushed.mp3': 'Finished pushing the box',
        'finished.mp3': 'Finished the course',
    }

    for n in range(-20, 20):
        file_contents['number_{}.mp3'.format(n)] = '{}'.format(n)

    for filename, text in file_contents.iteritems():
        gTTS(text=text).save(filename)

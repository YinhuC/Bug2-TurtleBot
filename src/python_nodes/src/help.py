# The script contains CR characters. The shell interprets these CR characters as arguments.
# Solution: Remove the CR characters from the script using the following script.
# https://stackoverflow.com/questions/19425857/env-python-r-no-such-file-or-directory

with open('takephoto.py', 'rb+') as f:
    content = f.read()
    f.seek(0)
    f.write(content.replace(b'\r', b''))
    f.truncate()

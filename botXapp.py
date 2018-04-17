from botXsrc.botXexport import botXexport
import time

"""
botXexport is a dictionary containing all the reusable components you
developed for the project, and you will use them in the main program.
"""
def main():
    print('starting app ...')
    grasp_api = botXexport['grasp_api']['module']()
    grasp_api.setup()
    time.sleep(5)
    grasp_api.object_to_grasp('cup')

"""
This is the only script that should be running from terminal so that the
program can gather modules correctly, so we need to specify main as entry point.
"""
if __name__ == '__main__':
    main()

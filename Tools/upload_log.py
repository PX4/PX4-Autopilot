#!/usr/bin/env python

"""
Upload an ULog file to the logs.px4.io web server.

@author: Beat Kueng (beat-kueng@gmx.net)
"""


from __future__ import print_function

from argparse import ArgumentParser
import subprocess
import sys


try:
    import requests
except:
    print("Failed to import requests.")
    print("You may need to install it with 'pip install requests'")
    print("")
    raise


SERVER = 'http://logs.px4.io'
#SERVER = 'http://localhost:5006' # for testing locally
UPLOAD_URL = SERVER+'/upload'

quiet = False

def ask_value(text, default=None):
    """ ask the user to provide a certain value """
    if quiet:
        return ""

    ask_string = 'Enter ' + text
    if default != None:
        ask_string += ' (Press ENTER to use ' + default + ')'
    ask_string += ': '

    if sys.version_info[0] < 3:
        ret = raw_input(ask_string)
    else:
        ret = input(ask_string)

    if ret == "" and default != None:
        return default
    return ret

def get_git_email():
    """ get (globally) configured git email """
    try:
        output = subprocess.check_output(["git", "config", "--global", "user.email"])
    except Exception:
        return ""

    return output.decode("utf-8").replace('\n', '')


def main():
    global quiet
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--quiet', '-q', dest='quiet', action='store_true', default=False,
            help='Quiet mode: do not ask for values which were not provided as parameters')
    parser.add_argument("--description", dest="description", type=str,
                      help="Log description", default=None)
    parser.add_argument("--feedback", dest="feedback", type=str,
                      help="Additional feedback", default=None)
    parser.add_argument("--source", dest="source", type=str,
                      help="Log source (Eg. CI)", default="webui")
    parser.add_argument("--email", dest="email", type=str,
                      help="Your e-mail (to send the upload link)", default=None)
    parser.add_argument("FILE", help="ULog file(s)", nargs="+")
    args = parser.parse_args()

    # arguments
    quiet = args.quiet
    if args.description == None:
        description = ask_value('Log Description')
    else:
        description = args.description

    if args.feedback == None:
        feedback = ask_value('Additional Feedback')
    else:
        feedback = args.feedback

    if args.email == None:
        default_email = get_git_email()
        email = ask_value('Your e-mail', default_email)
    else:
        email = args.email

    payload = {'type': 'personal', 'description': description,
            'feedback': feedback, 'email': email, 'source': args.source}

    for file_name in args.FILE:
        if not quiet:
            print('Uploading '+file_name+'...')

        with open(file_name, 'rb') as f:
            r = requests.post(UPLOAD_URL, data=payload, files={'filearg': f},
                    allow_redirects=False)
            if r.status_code == 302: # redirect
                if 'Location'  in r.headers:
                    plot_url = r.headers['Location']
                    if len(plot_url) > 0 and plot_url[0] == '/':
                        plot_url = SERVER + plot_url
                    print('URL: '+plot_url)


if __name__ == '__main__':
    main()


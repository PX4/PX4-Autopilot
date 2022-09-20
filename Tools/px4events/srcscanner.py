import os
import re
import codecs
import sys

class SourceScanner(object):
    """
    Traverses directory tree, reads all source files, and passes their contents
    to the Parser.
    """

    def ScanDir(self, srcdirs, parser):
        """
        Scans provided path and passes all found contents to the parser using
        parser.Parse method.
        """
        extensions = tuple([".cpp"])
        for srcdir in srcdirs:
            if os.path.isfile(srcdir):
                if not self.ScanFile(srcdir, parser):
                    return False
            else:
                for dirname, dirnames, filenames in os.walk(srcdir):
                    for filename in filenames:
                        if filename.endswith(extensions):
                            path = os.path.join(dirname, filename)
                            try:
                                if not self.ScanFile(path, parser):
                                    return False
                            except:
                                print(("Exception in file %s" % path))
                                raise
        return True

    def ScanFile(self, path, parser):
        """
        Scans provided file and passes its contents to the parser using
        parser.Parse method.
        """

        with codecs.open(path, 'r', 'utf-8') as f:
            try:
                contents = f.read()
            except:
                contents = ''
                print('Failed reading file: %s, skipping content.' % path)
                pass
        try:
            return parser.Parse(contents, path)
        except Exception as e:
            print("Exception while parsing file {}".format(path))
            raise

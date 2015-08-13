import os
import re
import codecs

class SourceScanner(object):
    """
    Traverses directory tree, reads all source files, and passes their contents
    to the Parser.
    """

    def ScanDir(self, srcdir, parser):
        """
        Scans provided path and passes all found contents to the parser using
        parser.Parse method.
        """
        extensions = tuple(parser.GetSupportedExtensions())
        for dirname, dirnames, filenames in os.walk(srcdir):
            for filename in filenames:
                path = os.path.join(dirname, filename)
                if not self.ScanFile(path, parser):
                    return False
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
        return parser.Parse(path, contents)

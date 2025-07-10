import sys
import re
import os

class AirframeGroup(object):
    """
    Airframe group

    type: specific vehicle type (e.g. VTOL Tiltrotor, VTOL Quadrotor, etc.)
    class: vehicle class (e.g. Multicopter, Fixed Wing, etc.)
    """
    def __init__(self, type, af_class):
        self.type = type
        self.af_class = af_class
        self.airframes = []


    def AddAirframe(self, airframe):
        """
        Add airframe to the airframe group
        """
        self.airframes.append(airframe)

    def GetType(self):
        """
        Get airframe group's vehicle type

        e.g. VTOL Tiltrotor, VTOL Quadrotor, etc.
        """
        return self.type

    def GetClass(self):
        """
        Get airframe group's vehicle class

        e.g. Multicopter, Fixed Wing, etc.
        """
        return self.af_class

    def GetImageName(self):
        """
        Get parameter group image base name (w/o extension)
        """
        if (self.type == "Standard Plane"):
            return "Plane"
        elif (self.type == "Flying Wing"):
            return "FlyingWing"
        elif (self.type == "Quadrotor x"):
            return "QuadRotorX"
        elif (self.type == "Quadrotor +"):
            return "QuadRotorPlus"
        elif (self.type == "Hexarotor x"):
            return "HexaRotorX"
        elif (self.type == "Hexarotor +"):
            return "HexaRotorPlus"
        elif (self.type == "Octorotor +"):
            return "OctoRotorPlus"
        elif (self.type == "Octorotor x"):
            return "OctoRotorX"
        elif (self.type == "Octorotor Coaxial"):
            return "OctoRotorXCoaxial"
        elif (self.type == "Octo Coax Wide"):
            return "OctoRotorXCoaxial"
        elif (self.type == "Quadrotor Wide"):
            return "QuadRotorWide"
        elif (self.type == "Quadrotor H"):
            return "QuadRotorH"
        elif (self.type == "Dodecarotor cox"):
            return "DodecaRotorXCoaxial"
        elif (self.type == "Simulation"):
            return "AirframeSimulation"
        elif (self.type == "Plane A-Tail"):
            return "PlaneATail"
        elif (self.type == "Plane V-Tail"):
            return "PlaneVTail"
        elif (self.type == "VTOL Duo Tailsitter"):
            return "VTOLDuoRotorTailSitter"
        elif (self.type == "Standard VTOL"):
            return "VTOLPlane"
        elif (self.type == "VTOL Quad Tailsitter"):
            return "VTOLQuadRotorTailSitter"
        elif (self.type == "VTOL Tiltrotor"):
            return "VTOLTiltRotor"
        elif (self.type == "VTOL Octoplane"):
            return "VTOLPlaneOcto"
        elif (self.type == "Coaxial Helicopter"):
            return "HelicopterCoaxial"
        elif (self.type == "Helicopter"):
            return "Helicopter"
        elif (self.type == "Hexarotor Coaxial"):
            return "Y6B"
        elif (self.type == "Y6A"):
            return "Y6A"
        elif (self.type == "Tricopter Y-"):
            return "YMinus"
        elif (self.type == "Tricopter Y+"):
            return "YPlus"
        elif (self.type == "Autogyro"):
            return "Autogyro"
        elif (self.type == "Airship"):
            return "Airship"
        elif (self.type == "Rover"):
            return "Rover"
        elif (self.type == "Boat"):
            return "Boat"
        elif (self.type == "Balloon"):
            return "Balloon"
        elif (self.type == "Vectored 6 DOF UUV"):
            return "Vectored6DofUUV"
        return "AirframeUnknown"

    def GetAirframes(self):
        """
        Returns the parsed list of airframes objects. Note that returned
        object is not a copy. Modifications affect state of the parser.
        """
        return sorted(self.airframes, key=lambda x: x.GetId())

class Airframe(object):
    """
    Single Airframe definition
    """

    # Define sorting order of the fields
    priority = {
		"board": 9,
        "short_desc": 8,
        "long_desc": 7,
        "min": 5,
        "max": 4,
        "unit": 3,
        "AUX1": -10,
        "AUX2": -10,
        "AUX3": -10,
        "AUX4": -10,
        "AUX5": -10,
        "AUX6": -10,
        "AUX7": -10,
        "AUX8": -10,
        # all others == 0 (sorted alphabetically)
    }

    def __init__(self, path, post_path, name, airframe_type, airframe_class, airframe_id, maintainer):
        self.fields = {}
        self.outputs = {}
        self.archs = {}
        self.path = path
        self.post_path = post_path
        self.name = name
        self.type = airframe_type
        self.af_class = airframe_class
        self.id = airframe_id
        self.maintainer = maintainer

    def GetPath(self):
        """
        Get path to airframe startup script
        """
        return self.path

    def GetPostPath(self):
        """
        Get path to airframe post startup script
        """
        return self.post_path

    def GetName(self):
        """
        Get airframe name
        """
        return self.name

    def GetType(self):
        """
        Get airframe type
        """
        return self.type

    def GetClass(self):
        """
        Get airframe class
        """
        return self.af_class

    def GetId(self):
        """
        Get airframe id
        """
        return self.id

    def GetMaintainer(self):
        """
        Get airframe maintainer
        """
        return self.maintainer

    def SetField(self, code, value):
        """
        Set named field value
        """
        self.fields[code] = value

    def SetOutput(self, code, value):
        """
        Set named output value
        """
        self.outputs[code] = value

    def SetArch(self, code, value):
        """
        Set named arch value
        """
        self.archs[code] = value

    def GetFieldCodes(self):
        """
        Return list of existing field codes in convenient order
        """
        keys = self.fields.keys()
        keys = sorted(keys)
        keys = sorted(keys, key=lambda x: self.priority.get(x, 0), reverse=True)
        return keys

    def GetFieldValue(self, code):
        """
        Return value of the given field code or None if not found.
        """
        fv =  self.fields.get(code)
        if not fv:
                # required because python 3 sorted does not accept None
                return ""
        return self.fields.get(code)

    def GetOutputCodes(self):
        """
        Return list of existing output codes in convenient order
        """
        keys = self.outputs.keys()
        keys = sorted(keys)
        keys = sorted(keys, key=lambda x: self.priority.get(x, 0), reverse=True)
        return keys

    def GetOutputValue(self, code):
        """
        Return value of the given output code or None if not found.
        """
        fv =  self.outputs.get(code)
        if not fv:
                # required because python 3 sorted does not accept None
                return ""
        return self.outputs.get(code)

    def GetArchCodes(self):
        """
        Return list of existing arch codes in convenient order
        """
        keys = self.archs.keys()
        keys = sorted(keys)
        keys = sorted(keys, key=lambda x: self.priority.get(x, 0), reverse=True)
        return keys

    def GetArchValue(self, code):
        """
        Return value of the given arch code or None if not found.
        """
        fv =  self.archs.get(code)
        if not fv:
                # required because python 3 sorted does not accept None
                return ""
        return self.archs.get(code)

class SourceParser(object):
    """
    Parses provided data and stores all found parameters internally.
    """

    re_split_lines = re.compile(r'[\r\n]+')
    re_comment_start = re.compile(r'^\#\s')
    re_comment_content = re.compile(r'^\#\s*(.*)')
    re_comment_tag = re.compile(r'@([a-zA-Z][a-zA-Z0-9_]*)\s*(.*)')
    re_comment_end = re.compile(r'(.*?)\s*\#\n/')
    re_cut_type_specifier = re.compile(r'[a-z]+$')
    re_is_a_number = re.compile(r'^-?[0-9\.]')
    re_remove_dots = re.compile(r'\.+$')
    re_remove_carriage_return = re.compile('\n+')

    valid_tags = set(["url", "maintainer", "output", "arch", "name", "type", "desc"])

    # Order of parameter groups
    priority = {
        # All other groups = 0 (sort alphabetically)
        "Miscellaneous": -10
    }

    def __init__(self):
        self.airframe_groups = {}

    def GetSupportedExtensions(self):
        """
        Returns list of supported file extensions that can be parsed by this
        parser. The parser uses any extension.
        """
        return ["", ".hil"]

    def Parse(self, path, contents):
        """
        Incrementally parse program contents and append all found airframes
        to the list.
        """

        airframe_id = None
        airframe_id = os.path.split(path)[1].split('_',1)[0]

        # Skip if not numeric
        if (not self.IsNumber(airframe_id)):
            return True

        # This code is essentially a comment-parsing grammar. "state"
        # represents parser state. It contains human-readable state
        # names.
        state = None
        tags = {}
        outputs = {}
        archs = {}
        for line in self.re_split_lines.split(contents):
            line = line.strip()
            # Ignore empty lines
            if line == "":
                continue
            if state is None and self.re_comment_start.match(line):
                state = "wait-short"
                short_desc = None
                long_desc = None
            if state is not None and state != "comment-processed":
                m = self.re_comment_end.search(line)
                if m:
                    line = m.group(1)
                    last_comment_line = True
                else:
                    last_comment_line = False
                m = self.re_comment_content.match(line)
                if m:
                    comment_content = m.group(1)
                    if comment_content == "":
                        # When short comment ends with empty comment line,
                        # start waiting for the next part - long comment.
                        if state == "wait-short-end":
                            state = "wait-long"
                    else:
                        m = self.re_comment_tag.match(comment_content)
                        if m:
                            tag, desc = m.group(1, 2)
                            if (tag == "output"):
                                key, text = desc.split(' ', 1)
                                outputs[key] = text
                            elif (tag == "board"):
                                key, text = desc.split(' ', 1)
                                archs[key] = text
                            else:
                                tags[tag] = desc
                            current_tag = tag
                            state = "wait-tag-end"
                        elif state == "wait-short":
                            # Store first line of the short description
                            short_desc = comment_content
                            state = "wait-short-end"
                        elif state == "wait-short-end":
                            # Append comment line to the short description
                            short_desc += "\n" + comment_content
                        elif state == "wait-long":
                            # Store first line of the long description
                            long_desc = comment_content
                            state = "wait-long-end"
                        elif state == "wait-long-end":
                            # Append comment line to the long description
                            long_desc += "\n" + comment_content
                        elif state == "wait-tag-end":
                            # Append comment line to the tag text
                            tags[current_tag] += "\n" + comment_content
                        else:
                            raise AssertionError(
                                    "Invalid parser state: %s" % state)
                elif not last_comment_line:
                    # Invalid comment line (inside comment, but not starting with
                    # "*" or "*/". Reset parsed content.
                    state = None
                if last_comment_line:
                    state = "comment-processed"
            else:
                state = None

        # Process parsed content
        airframe_type = None
        maintainer = None
        airframe_name = None
        airframe_class = None

        # Done with file, store
        for tag in tags:
            if tag == "maintainer":
                maintainer = tags[tag]
            elif tag == "type":
                airframe_type = tags[tag]
            elif tag == "class":
                airframe_class = tags[tag]
            elif tag == "name":
                airframe_name = tags[tag]
            elif tag == "desc":
                # General documentation - not a parameter to be saved.
                pass
            elif tag not in self.valid_tags:
                sys.stderr.write("Aborting due to invalid documentation tag: '%s'\n" % tag)
                return False

        # Sanity check
        if airframe_type == None:
            sys.stderr.write("Aborting due to missing @type tag in file: '%s'\n" % path)
            return False

        if airframe_class == None:
            sys.stderr.write("Aborting due to missing @class tag in file: '%s'\n" % path)
            return False

        if airframe_name == None:
            sys.stderr.write("Aborting due to missing @name tag in file: '%s'\n" % path)
            return False

        # Check if a .post script exists
        if os.path.isfile(path + '.post'):
            post_path = path + '.post'
        else:
            post_path = None

        # We already know this is an airframe config, so add it
        airframe = Airframe(path, post_path, airframe_name, airframe_type, airframe_class, airframe_id, maintainer)

        # Done with file, store
        for tag in tags:
            if tag == "maintainer":
                maintainer = tags[tag]
            if tag == "type":
                airframe_type = tags[tag]
            if tag == "class":
                airframe_class = tags[tag]
            if tag == "name":
                airframe_name = tags[tag]
            else:
                airframe.SetField(tag, tags[tag])

        # Store outputs
        for output in outputs:
            airframe.SetOutput(output, outputs[output])

        # Store outputs
        for arch in archs:
            airframe.SetArch(arch, archs[arch])

        # Store the parameter

        # Create a class-specific airframe group. This is needed to catch cases where an airframe type might cross classes (e.g. simulation)
        class_group_identifier=airframe_type + airframe_class
        if class_group_identifier not in self.airframe_groups:
            #self.airframe_groups[airframe_type] = ParameterGroup(airframe_type)  #HW TEST REMOVE
            self.airframe_groups[class_group_identifier] = AirframeGroup(airframe_type, airframe_class)
        self.airframe_groups[class_group_identifier].AddAirframe(airframe)

        return True

    def IsNumber(self, numberString):
        try:
            float(numberString)
            return True
        except ValueError:
            return False

    def Validate(self):
        """
        Validates the airframe meta data.
        """
        seenParamNames = []
        for group in self.GetAirframeGroups():
            for param in group.GetAirframes():
                name  = param.GetName()
                board = param.GetFieldValue("board")
                # Check for duplicates
                name_plus_board = name + "+" + board
                for seenParamName in seenParamNames:
                    if seenParamName == name_plus_board:
                        sys.stderr.write("Duplicate parameter definition: {0}\n".format(name_plus_board))
                        return False
                seenParamNames.append(name_plus_board)

        return True

    def GetAirframeGroups(self):
        """
        Returns the parsed list of Airframe groups. Every Airframe is an Airframe
        object. Note that returned object is not a copy. Modifications affect
        state of the parser.
        """
        groups = self.airframe_groups.values()
        groups = sorted(groups, key=lambda x: x.GetType())
        groups = sorted(groups, key=lambda x: x.GetClass())
        groups = sorted(groups, key=lambda x: self.priority.get(x.GetType(), 0), reverse=True)

        #Rename duplicate groups to include the class (creating unique headings in page TOC)
        duplicate_test=set()
        duplicate_set=set()
        for group in groups:
            if group.GetType() in duplicate_test:
                duplicate_set.add(group.GetType())
            else:
                duplicate_test.add(group.GetType() )
        for group in groups:
            if group.GetType() in duplicate_set:
                group.name=group.GetType()+' (%s)' % group.GetClass()

        return groups

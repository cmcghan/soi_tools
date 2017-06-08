#!/usr/bin/python
# Copyright © 2017 Government of the United States of America, as represented by the Secretary of the Air Force.
# No copyright is claimed in the United States under Title 17, U. S. Code. All Other Rights Reserved.
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/cmcghan/soi_tools
# Additional copyright may be held by others, as reflected in the commit history.
#
# Abbreviated changelog:
# 2017-06-08: modified from soi_tools/lmcp2aadl/lmcp2aadl.py
# 2017-05-30: first created (copy of soi_tools/lmcp2aadl/lmcp2aadl.py)

# for lxml, should be installed (except slt), but can install: (see: http://lxml.de/installation.html )
# sudo apt-get install libxml2-dev libxslt-dev python-dev

#import xml.etree.ElementTree as ET
import lxml.etree as ET
import argparse as AP
import sys
import textwrap
import os

def strip_whitespace(comment):
    if comment != None:
        return ' '.join(comment.split())
    else:
        return None

# prev. was "class Data_Type"
class StructInfo: # StructInfo; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/StructInfo.java
    def __init__(self, name, extends, series, comment, fields):
        self.name = name #public String name = "";
        #public int id = 0;
        self.extends = extends #public String extends_name = "";
        self.series = series # new #public String extends_series = "";
        self.comment = comment # new #public String comment = "";
        self.fields = fields #public FieldInfo[] fields = new FieldInfo[0];
        # handled in MDM / main XMLCollector below #public String seriesName = "";
        # handled in MDM / main XMLCollector below #public String namespace = "";

    def __str__(self): # *** WIP ***
        #s = self.name + ': \n'
        s = self.name + ': (extends: ' + str(self.extends) + ', series: ' + str(self.series) + ')\n'
        s += '  comment: ' + str(self.comment) + '\n'
        for field in self.fields:
            #s += '    ' + str(field) + ' \n'
            s += '    ' + str(field) + ' \n'
        return s

    #def set_comment(self, text):
    #    self.comment = strip_whitespace(text)
        
    __repr__ = __str__

class FieldInfo: # FieldInfo; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/FieldInfo.java
    def __init__(self, name, type, series, comment, defaultVal, units):
        self.name = name #public String name = "";
        self.comment = comment #public String comment = "";
        self.length = 1 # new #public int length = 1;
        self.type = type #public String type = "";
        self.series = series # new # *** not part of spec, but is generally given (?!?!?!) ***
        self.defaultVal = defaultVal # new #public String defaultVal = "";
        self.units = units # new #public String units = "";
        # handled in MDM / main XMLCollector below # new #public String seriesName = "";
        ##self.is_Scalar = False # new #public boolean isScalar = false;
        ##self.is_array = False #public boolean isArray = false;
        self.is_Array = False #public boolean isArray = false;
        ##self.is_LargeArray = False # new #public boolean isLargeArray = false;
        ##self.is_Enum = False # new #public boolean isEnum = false;
        ##self.is_Struct = False # new #public boolean isStruct = false;
        ##self.is_Map = False # new #public boolean isMap = false;
        self._translate_array_fields() # called automatically on initialization; would need to call this manually if setting self.type after constructor-call
        
    # for ROS .msg file, this stripping process is unnecessary
    def _translate_array_fields(field): # "field" is "self", here...
        if field.type.endswith('[]'):
    #        print("type = %r" % field.type)
    #        field.type = field.type[:-2]
    #        #field.is_array = True;
    #        field.length = 0 # new # unknown length
    #        print("length = %r" % field.length)
            field.is_Array = True;

    def __str__(self):
        str_ret = self.name + ' : ' + self.type
        if self.series != None:
            str_ret += ' in series = ' + str(self.series)
        if self.units != None:
            str_ret += ' (units = ' + str(self.units) + ')'
        if self.defaultVal != None:
            str_ret += ' (defaultVal = ' + str(self.units) + ')'
        if self.comment != None:
            str_ret += ' (' + str(self.comment) + ')\n'
        return str_ret

    #def set_comment(self, text):
    #    self.comment = strip_whitespace(text)

    __repr__ = __str__

class EnumInfo: # EnumInfo; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/EnumInfo.java
    def __init__(self, name, entries, comment):
        self.name = name #public String name = "";
        self.comment = comment # new #public String comment = "";
        # new #public String namespace = "";
        # new #public String seriesName = "";
        #self.vals = vals
        self.entries = entries #public ArrayList<EnumEntry> entries = new ArrayList<EnumEntry>();
        #public static class EnumEntry {
        #    public String name = "";
        #    public String value = "0";
        #    public String comment = "";
        #}

    #def number_the_enums(): # need to give numbers to these for ROS msg stuff (setting constants)
    #    any_not_numbered = False
    #    # if following C++ convention for enumerated types:
    #    if self.entries != None:
    #        i = 0
    #        for j in range(len(self.entries)):
    #            if self.entries[j].value is None:
    #                self.entries[j].value = i
    #            else:
    #                i = self.entries[j].value
    #            i += 1
    #
    #    if self.entries != None:
    #        for entry in self.entries:
    #            if entry.value is None:
    #                any_not_numbered = True
    #        # way to handle this #1: number them
    #        if any_not_numbered == True:
    #            for i in range(len(self.entries)):
    #                self.entries[i].value = i
    #        # way to handle this #2: turn them into strings
    #        if any_not_numbered == True:
    #            for i in range(len(self.entries)):
    #                self.entries[i].value = self.entries[i].name # give it the same value as the string name

    def __str__(self):
        str_ret = self.name
        if self.entries != None:
            str_ret += ' : ' + str(self.entries)
        if self.comment != None:
            str_ret += ' (' + str(self.comment) + ')\n'
        return str_ret

    #def set_comment(self, text):
    #    self.comment = strip_whitespace(text)
        
    __repr__ = __str__

class EnumEntry: # new # EnumEntry; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/EnumInfo.java
    def __init__(self, name, value, comment):
        #public static class EnumEntry {
        #    public String name = "";
        #    public String value = "0";
        #    public String comment = "";
        #}
        self.name = name #public String name = "";
        # enumerated types can be strings (of themselves) if no value explicitly given!
        # see: http://en.cppreference.com/w/cpp/language/enum
        # and for the C++11 way: https://stackoverflow.com/a/24296298
        self.is_Value = False # needed for ROS msg
        self.is_String = False # needed for ROS msg
        if value != None:
            self.value = value # set to value-given
            self.is_Value = True # will assume is a uint8 here
        else:
            self.value = name # if no value given, set it to the name string (for ROS)
            self.is_String = True
        self.comment = comment # new #public String comment = "";
        
    def __str__(self):
        str_ret = self.name
        if self.value != None:
            str_ret += ' : ' + str(self.value)
        if self.comment != None:
            str_ret += ' (' + str(self.comment) + ')'
        return str_ret

    #def set_comment(self, text):
    #    self.comment = strip_whitespace(text)
        
    __repr__ = __str__

# basing data types off of: LmcpGen/src/avtas/lmcp/lmcpgen/PythonMethods.java
#private static String getPythonType(String type) {
#if (type.toLowerCase().matches("(bool)")) {

def grabPieceArray(type):
    # [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(type)
#LXML bool -> ROS bool (-> Python bool)
#LXML string -> ROS string (-> Python str)
#LXML char -> ROS string (-> Python str)
#LXML byte -> ROS uint8 (-> Python int)
#LXML int64 -> ROS int64 (-> Python int)
#LXML int32 -> ROS int32 (-> Python int)
#LXML uint32 -> ROS uint32 (-> Python int)
#LXML int16 -> ROS int16 (-> Python int)
#LXML uint16 -> ROS uint16 (-> Python int)
#LXML real32 -> ROS float32 (-> Python float)
#LXML real64 -> ROS float64 (-> Python float)

# ROS int8 -> LXML ??
# ROS uint64 -> LXML ??
# ROS time (secs/nsecs uint32) -> LXML ??
# ROS duration (secs/nsecs uint32) -> LXML ??

    lxml_to_ros_dict = {"bool": "bool", "string": "string", "char": "string",
                        "byte": "uint8", "int64": "int64", "int32": "int32",
                        "uint32": "uint32", "int16": "int16", "uint16": "uint16",
                        "real32": "float32", "real64": "float64"}
    
    # get last / location
    ii = 0; previi = 0
    typehold = str(type)
    while ii != -1:
        previi = ii
        ii = typehold.find("/")
        typehold = typehold[ii+1:len(typehold)]
    # split at "/", leaving out "/"
    if previi == 0: # no split required
        typepre = ''
        typestr = str(type)
    else: # need to split
        typepre = type[0:previi]
        typestr = type[previi+1:len(type)] # removes "/"
    
    # get array portion if exists, split out array if exists
    holdindex = typestr.find("[")
    if holdindex != -1:
        type_piece = typestr[0:holdindex]
        type_array = typestr[holdindex:len(typestr)]
    else:
        type_piece = typestr
        type_array = ""
    
    return [typepre,type_piece,type_array,lxml_to_ros_dict]

def hasBasicRosType(type):
    [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(type)
    
    if (type_piece.lower() in lxml_to_ros_dict):
        return True
    else:
        return False

def getRosType(type): # type assumed to be a string
    [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(type)
    
    if (type_piece.lower() in lxml_to_ros_dict):
        holdstr = lxml_to_ros_dict[type_piece.lower()]
    else:
        holdstr = type_piece

    ## ROS uses same array [] or [#] as LXML, so just find-replace portions of the string as-needed
    #hold = type.replace("char","string",1)
    #hold = hold.replace("byte","uint8",1)
    #hold = hold.replace("real","float",1)
    ## this won't work if part of a created type includes these, though...
    if typepre !=  '':
        return typepre+"_msgs/"+holdstr+type_array
    else:
        return holdstr+type_array
    

def name_series_fixer(name,series)
    
    
    return [fixedname,fixedseries]

def extends_series_fixer(extends,series)
    
    
    return [fixedextends,fixedseries]

class XMLCollector_rosmsg(object): # MDMInfo; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/MDMInfo.java
    def __init__(self):
        self.MDMseriesName = None # new #public String seriesName = "";
        self.MDMnamespace = None # new #public String namespace = "";
        self.MDMcomment = None # new #public String comment = "";
        self.structs = [] #public StructInfo[] structs = new StructInfo[0];
        self.enums = [] #public EnumInfo[] enums = new EnumInfo[0];
        #new #public String mdmString = ""; #/** a string containing the contents of the MDM file */
        self.MDMversion = 0 # new #public int version = 0;
        self.seriesNameAsLong = 0 # new #public long seriesNameAsLong = 0;
        
        #self.cur_el = None
        self.cur_str = None # new
        self.cur_enum = None # new
        self.cur_field = None # new
        self.cur_fields = None
        self.cur_entry = None # new
        self.cur_entries = None
        self.cur_comment = None
        self.cur_data = None # new
        
    def start(self, tag, attrib):
        #print("Start:")
        #print("tag: %r" % tag)
        #print("attrib: %r " % dict(attrib))
        if tag == "MDM" : # new
            #print("MDM attrib: ")
            #print(dict(attrib)) # see: http://lxml.de/tutorial.html
            # text? tail?
            if self.cur_comment != None:
                self.MDMcomment = self.cur_comment
                self.cur_comment = None
                #print("MDM comment: %r" % self.MDMcomment)
        if tag == "Struct" : # --> Field
            #print("start Struct %r" % tag)
            name = attrib['Name']
            extends = None
            series = None # new
            if 'Extends' in attrib:
                extends = attrib['Extends']
            if 'Series' in attrib: # new
                series = attrib['Series']
            #[extends,series] = extends_series_fixer(extends,series)
            self.cur_fields = []
            #self.cur_el = Data_Type(name, extends, self.cur_fields)
            self.cur_str = StructInfo(name, extends, series, self.cur_comment, self.cur_fields)
            self.cur_comment = None # new
            #print("StructInfo made")
            #self.structs.append(self.cur_el)
        if tag == "Field" :
            #print("start Field %r" % tag)
            name = attrib['Name']
            field_type = attrib['Type']
            series = None
            if 'Series' in attrib: # new
                series = attrib['Series']
                #[name,series] = name_series_fixer(name,series)
            defaultVal = None
            units = None
            if 'Default' in attrib:
                defaultVal = attrib['Default']
            if 'Units' in attrib: # new
                units = attrib['Units']
            #self.cur_el = Field(name, field_type)
            self.cur_field = FieldInfo(name, field_type, series, self.cur_comment, defaultVal, units)
            self.cur_comment = None # new
            self.cur_fields.append(self.cur_field)
            #print("FieldInfo made+appended")
        if tag == "Enum" : # --> Entry
            #print("start Enum %r" % tag)
            name = attrib['Name']
            self.cur_entries = []
            #self.cur_el = Enum(name, self.cur_entries)
            self.cur_enum = EnumInfo(name, self.cur_entries, self.cur_comment)
            self.cur_comment = None # new
            #print("EnumInfo made")
            #self.enums.append(self.cur_el)
        if tag == "Entry":
            #print("start Entry %r" % tag)
            name = attrib['Name']
            value = None
            if 'Value' in attrib:
                value = attrib['Value']
            self.cur_entry = EnumEntry(name,value,self.cur_comment)
            self.cur_comment = None
            self.cur_entries.append(self.cur_entry)
            #print("Entry (made+)appended")
        #if self.cur_comment != None and self.cur_el != None:
        #    self.cur_el.set_comment(self.cur_comment)
        #if self.cur_data != None and self.cur_el != None:
        #    self.cur_el.set_data(self.cur_data)
        
    def end(self, tag): # don't have data to go with tags until end-tag
        #print("End:")
        #print("tag: %r" % tag)
        if tag == "SeriesName" : # new
            self.MDMseriesName = strip_whitespace(self.cur_data)
            #print("self.seriesName = %r" % self.seriesName)
        if tag == "Namespace" : # new
            self.MDMnamespace = strip_whitespace(self.cur_data)
            #print("self.namespace = %r" % self.namespace)
        if tag == "Version" : # new
            self.MDMversion = float(strip_whitespace(self.cur_data))
            #print("self.version = %r" % self.version)
        if tag == "Struct" : # --> Field
            self.cur_str.fields = self.cur_fields
            #self.cur_fields = []
            self.structs.append(self.cur_str)
            #print("end Struct %r" % tag)
        #if tag == "Field" :
            #print("end Field %r" % tag)
        if tag == "Enum" : # --> Entry
            #self.cur_el = Enum(name, self.cur_entries)
            self.cur_enum.entries = self.cur_entries
            #self.cur_entries = []
            self.enums.append(self.cur_enum)
            #print("end Enum %r" % tag)
        #if tag == "Entry":
            #print("end Entry %r" % tag)
        self.cur_comment = None
        self.cur_data = None # new
        
    def data(self, data): # new # see: http://lxml.de/parsing.html#the-target-parser-interface
        self.cur_data = data
        #print("Data: %r" % data) # note: this will give back unicode strings

    def comment(self, text): # new # do NOT set a var in __init__ to same name with =None, will wipe out f(n)!!
        self.cur_comment = strip_whitespace(text)
        #print("Comment: %r" % text)
        #print("Comment: %r" % self.cur_comment)

    def close(self):
        return

def find_local_pkg_path(dirname_to_find):
    # reference: https://stackoverflow.com/a/2186565
    import fnmatch
    import os
    matches_found = []
    for localroot, dirnames, filenames in os.walk('./'):
        for dirname in fnmatch.filter(dirnames, dirname_to_find):
            matches_found.append(os.path.join(localroot, dirname))
    
    #print("%s , %s" % (dirname_to_find, matches_found))
    if matches_found == []:
        print("DEBUG: error, found no matching subpackage for '%s'!" % dirname_to_find)
        return ""
    elif len(matches_found) == 1:
        return matches_found[0] # should only match the once
    else:
        print("DEBUG: error, found multiple subpackages for '%s'! (%r)" % (dirname_to_find,matches_found))
        return ""

def main():
    #parse all of the arguments
    ap = AP.ArgumentParser(description='Converts an LMCP xml file into AADL datatypes')
    ap.add_argument('runtype', metavar='runoption', type=str, choices=["file","dir"], help='the option "dir" or "file" that decides the run (dir for directory as input_file)')
    ap.add_argument('input_file', metavar='input', type=str, help='the input LMCP xml file')
    ap.add_argument('output_file', metavar='output', type=str, default=None, help='the output AADL file')

    args = ap.parse_args()
    args = vars(args)

    runtype = args.get('runtype')
    in_file = args.get('input_file')
    out_file = args.get('output_file')
    
    if in_file is None:
        print("Input filename not given. Exiting.")
        sys.exit(1)
    if out_file is None:
        print("Output filename not given. Exiting.")
        sys.exit(1)
    if in_file == out_file:
        print('the input file and output file must be different names')
        sys.exit(1)

    if runtype == "dir": # chaining multiple things together
        handle_all_files(in_file) # (xmldirstr)
    elif runtype == "file": # old/original way to do it
        read_XML_MDMs_file_and_output_to_file(in_file,out_file)
    else:
        print("ERROR: Unknown input switch, exiting.")
        sys.exit(1)

def parse_XML(in_file):
    #begin parsing xml
    contents = open(in_file, 'r').read()
    #print("contents: ")
    #print(contents)
    #print("\n")
    collector = XMLCollector_rosmsg()
    parser = ET.XMLParser(target = collector)
    result = ET.XML(contents, parser)
    #result = ET.parse(in_file, parser) # alt.
    
    #return [contents,collector,parse,result]
    return collector

def read_XML_MDMs_file_and_output_to_file(in_file,out_file=None):
    if in_file is None:
        print("Input filename not given. Exiting.")
        sys.exit(1)
    if out_file is None:
        print("Output filename not given. Will only write to individual files.")
    if in_file == out_file:
        print('the input file and output file must be different names')
        sys.exit(1)

    collector = parse_XML(in_file)
    
    # *** FOR DEBUGGING: ***
    #print("seriesName = %r" % collector.MDMseriesName)
    #print("namespace = %r" % collector.MDMnamespace)
    #print("version = %r" % collector.MDMversion)
    #print("comment = %r" % collector.MDMcomment)
    #print("structs = %r" % collector.structs)
    #print("enums = %r" % collector.enums)
    
    # package name is detemrined by collector.MDMseriesName, collector.MDMnamespace
    # output filenames for each struct is determined from collector.structs.name, collector.structs.series
    rospkgname = str(collector.MDMnamespace).lower() + "_msgs"
    #rospkgname = str(collector.seriesName).lower() + "_msgs" # this won't handle the "namespaces" well
    # use collector.MDMcomment for short README.txt document in repo
    os.system('mkdir -p %s' % str(rospkgname))
    readme_str = "##Series Name\n" + str(collector.MDMseriesName) + '\n'
    readme_str += "##Namespace\n" + rospkgname + '\n'
    readme_str += "##Version\n" + str(collector.MDMversion) + '\n'
    readme_str += "<hr>\n"
    readme_str += str(collector.MDMcomment) + '\n'
    out = open("./" + rospkgname + "/README.md", 'w')
    out.write(readme_str)
    out.close()
    
    if out_file is not None:
        out = open(out_file,'w')
    # output filenames for each struct is determined from collector.structs.name, collector.structs.series
    for s in collector.structs: # struct_to_rosmsg will internally write to individual files, but below line...
        [structstr,dependencies] = struct_to_rosmsg(s,collector.MDMseriesName,collector.MDMnamespace,rospkgname,collector.structs,collector.enums)
        if out_file is not None:
            out.write(structstr) # ...(also) writes to full file
            out.write("\n")
            out.write("# ----------------------------------------\n")
            out.write("# ----------------------------------------\n")
            out.write("# ----------------------------------------\n")
            out.write("\n")
        #if dependencies != []:
        #    print("%s/%s has external dependencies: %r" % (rospkgname,str(s.name),dependencies))
    
    #for e in collector.enums:
    #    out.write(enum_to_rosmsg(e))
    if out_file is not None:
        out.close()

def enum_to_rosmsg(enum):
    #
    # note: we are going to have to handle all the enums first, because
    # they need to be written into the corresponding .msg files at-need
    #
    ret = '\n'
    
    # EnumInfo
    # {name , comment , entries}
    s = "# Enum: " + enum.name + ret
    #print("Enum name = %s" % enum.name)
    if enum.comment != None:
        s += "# " + enum.comment.replace("\n","\n# ") + ret
    for entry in enum.entries:
        # EnumEntry
        #{name , is_Value , is_String , value , comment}
        if entry.comment != None:
            s += "# " + entry.comment.replace("\n","\n# ") + ret
        if entry.is_Value:
            s += "uint8 "
            enumtype="unit8" # should be consistent for all entries
        elif entry.is_String:
            s += "string "
            enumtype="string" # should be consistent for all entries
        else:
            s += "# ERROR: (--None--) "
            print("DEBUG: unknown enum entry type %s in enum %s" % (entry.name,enum.name))
        s += entry.name + "=" + entry.value + ret
    #print("enum s = %s" % s)
    return [s,enumtype]


#isLocalEnumType(struct.fields[i].type,enums)
def indexLocalEnumType(type,enums_list):
    index = -1
    # find if enum if in this series
    holdit_enum_names = [str(xx.name) for xx in enums_list]
    #print(holdit_enum_names)
    [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(type)
    #print("type: '%s' , type_piece: '%s'" % (str(type),str(type_piece)))
    if str(type_piece) in holdit_enum_names:
        # get index
        index = holdit_enum_names.index(str(type_piece))
    return index

#isOtherStructType(struct.fields[i].type,struct)
#def isOtherStructType(type,struct):
#    boolAns = False
#    
#    
#    
#    return boolAns

def handle_all_files(xmldirstr):
    # get all XML MDM filenames+paths together
    
    # reference: https://stackoverflow.com/a/2186565
    import fnmatch
    import os
    matches_found = []
    filename_list = os.listdir(xmldirstr)
    for filename in fnmatch.filter(filename_list, "*.xml"):
        matches_found.append(os.path.join(xmldirstr, filename))
    
    # now that we have every XML MDMs file in a list...
    # figure out what dependencies they have on each other
    unordered_list = []
    for filename in matches_found:
        c = parse_XML(in_file) # c as in collector
        dependencies = get_all_xml_file_deps(c.MDMseriesName,c.MDMnamespace,c.structs,c.enums)
        if dependencies == None:
            sys.exit(0)
        unordered_list.append([filename,str(c.MDMseriesName).lower() + "_msgs",dependencies])
    
    # then get them 'sorted' in the order in which dependencies need to be handled
    i = 0
    sorted_list = []
    requires_sorting = []
    for i in range(len(unordered_list)):
        if unordered_list[i][2] == []: # if dependencies == []
            sorted_list.append(unordered_list[i])
            deps_added.append(unordered_list[i][1])
        else: # add to requires-sorting part
            requires_sorting.append(unordered_list[i])
    print("sorted_list: %r" % sorted_list)
    print("deps_added: %r" % deps_added)
    print("requires_sorting: %r" % requires_sorting)
    i = 0
    prev = None
    holdsorting = None
    while (1):
        if i >= len(requires_sorting):
            # if there has been no change between holdsorting and requires_sorting through entire list
            # then there is a problem
            if (holdsorting is not None) and (holdsorting == requires_sorting):
                print("DEBUG: problem! conflicts with sorting list via dependencies")
                sys.exit(1)
            # clean out the old entries that were marked with 'None'
            holdsorting = []
            for j in range(len(requires_sorting)):
                if requires_sorting[j] is not None:
                    holdsorting.append(requires_sorting[j])
            # refresh requires_sorting
            requires_sorting = holdsorting
            # and reset counter
            i = 0
        if len(requires_sorting) == 0: # we're done!
            break
        #elif len(requires_sorting) == 1: # we only have one left, add to the end
        #    sorted_list.append(requires_sorting[0])
        #    break
        if len(requires_sorting) > 0:
            #if requires_sorting[i][1] == []: # if no dependencies, then just add to the list
            #    sorted_list.append(requires_sorting[i]) # this is commented out because this was already done!
            in_there = 1
            for depname in requires_sorting[i][2]:
                if not(depname in deps_added):
                    in_there = 0
                    break
            if in_there == 1:
                sorted_list.append(requires_sorting[i])
                deps_added.append(requires_sorting[i][1])
                requires_sorting[i] = None # "zero out" entry
        i = i+1
    
    # now that everything's sorted in an order that'll  be able to handle all deps, get each file handled!
    for i in range(len(sorted_list)):
        in_file = sorted_list[i][0]
        read_XML_MDMs_file_and_output_to_file(in_file,out_file=None)
    # after this, should be done!

def get_all_xml_file_deps(MDMseriesName,MDMnamespace,structs_list,enums_list):
    dependencies = []
    for s in structs_list:
        dependencies = get_struct_deps(s,MDMseriesName,MDMnamespace,enums_list,dependencies=[])
        if dependencies is None:
            print("DEBUG: problem with finding dependencies for %s!"  % s.name)
            return None
    return dependencies

def get_struct_deps(struct,MDMseriesName,MDMnamespace,enums_list,dependencies=[]):
    # struct.{name,extends,series,comment,fields}
    # fields.{name,comment,type,defaultVal,units}
    
    if struct.extends != None:
        #s += ret
        if struct.series != None:
            holdss = str(struct.series).lower() + "_msgs/"
        else:
            holdss = ""
        
        # find other struct
        if holdss == "" or holdss == str(str(MDMnamespace)+"_msgs/") or holdss == str(str(MDMseriesName).lower()+"_msgs/"): # then in this series (same MDM file)
            # (1) find and grab from other struct if in this series
            # other struct will show an external dependency when we get to it, will be added then
            # we only really care about global pkg deps per-XML MDMs file
            pass
        else: # then this must be from some other file
            # (2a) open and read file if in an entirely different series
            #print("*** need to grab this from another file ***")
            if struct.series != None:
                dep = str(struct.series).lower() + "_msgs"
                dep_fixed = find_local_pkg_path(dep)
                if dep_fixed != "":
                    pass
                else: # problem with finding directory
                    return None
            else: # series not given, need to figure out what external file this came from
                [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(struct.extends)
                #ii = struct.extends.find("/")
                if typepre != None:
                    dep = str(typepre).lower() + "_msgs"
                    dep_fixed = find_local_pkg_path(dep)
                    if dep_fixed != "":
                        pass
                    else: # problem with finding directory
                        return None
                else: # this should not happen...
                    print("DEBUG: error, name %s 'extends' (%s) should include '/' in this circumstance, failing up..." % (struct.name,struct.extends))
                    return None
            # (2b) add to pkg msg dependency list for pkg if needed other pkg
            if not(dep in dependencies):
                dependencies.append(dep)
            #print("dependencies for %s %s: %r" % (rospkgname,struct.name,dependencies))
    
    for i in range(len(struct.fields)):
        # trying to get pkg/series to go with type...
        if not hasBasicRosType(struct.fields[i].type):
            enumindex = indexLocalEnumType(struct.fields[i].type,enums_list)
            if enumindex != -1: # then need to find and add enum here
                pass
            else:
                if (struct.fields[i].series != None) and (struct.fields[i].series != ""): # ...this should be enough, have to give remote series
                    dep = str(struct.fields[i].series).lower() + "_msgs/"
                    s += dep
                    if not(dep in dependencies):
                        dependencies.append(dep)
                else: # assuming is inside current local set... probably a good assumption
                    pass
        else:
            pass
    
    return dependencies # None is returned if there is a problem, [] is returned if no external deps

def struct_to_rosmsg(struct,MDMseriesName,MDMnamespace,rospkgname,structs_list,enums_list,dependencies=[],level=0):
    #
    # note: "extends" is making a new class with ': public extended_class',
    # so the data names and datatypes are handled at the same level in UxAS
    # ...thus, if we want this to be equivalent, ROS msg should copy the same data-vars
    # into the new msg file, and -not- just lazily add a new var that uses that dataype as a subdata-var
    #
    # thus, we will need to order the msg creation such that the 'non-extends'
    # happen first, and the 'extends' happen in a workable order later on already-created files
    #
    
    #
    # <Struct Name="RadioState" Extends="PayloadState" Series="CMASI" >
    # --OR--
    # <Struct Name="TrackEntityAction" Extends="CMASI/VehicleAction">
    # -> the "<!-- -->" directly before this should be a comment at the start / for the entire .msg file
    # -> "Struct Name" is the filename of the .msg file
    # -> "Extends" may include "Series" info...
    # -> "Series" should be "series_msgs" pkg reference to another file that needs to be INCLUDED inside this one
    # -> "Extends" portion should be pointing to the (pkg_msgs/Message.msg) file for INCLUSION inside this one
    #
    # <!-- The unique identifier of the target to be tracked. -->
    #        <Field Name="EntityID" Type="uint32"  />
    # -> the "<!-- -->" directly before a new "<...>" should be set as a comment on that piece in the .msg file
    # -> "Field "Type" is the datatype (comes first on the line)
    # -> "Field Name" is the name of the variable (comes second on the line)
    #
    # <Field Name="Latitude" Type="real64" Units="degree" />
    # -> 
    #
    # <Field Name="AltitudeType" Type="AltitudeType" Default="MSL" />
    # -> "Field Default" is the initial value to set this to... except that ROS doesn't support this
    #    (...does 0mq??)
    #
    
    # struct.{name,extends,series,comment,fields}
    # fields.{name,comment,type,defaultVal,units}
    
    if (level>100):
        print("It looks like we went down the rabbit hole on 'extends'.")
        return ""
    
    ret = "\n"
    
    outfile = "./" + rospkgname + "/" + struct.name + ".msg"
    
    s = "# Struct: " + struct.name + ret
    if struct.comment != None:
        s += "# " + struct.comment.replace("\n","\n# ") + ret
    if struct.extends != None:
        #s += ret
        if struct.series != None:
            holdss = str(struct.series).lower() + "_msgs/"
        else:
            holdss = ""
        s += "# Extends: " + holdss + struct.extends + ret # need to find and add all parts of other struct here ***WIP***
        s += "# ----------------------------------------" + ret
        s += "# ----------------------------------------" + ret
        
        extendshandled = 0
        # find other struct
        if holdss == "" or holdss == str(str(MDMnamespace)+"_msgs/") or holdss == str(str(MDMseriesName).lower()+"_msgs/"): # then in this series (same MDM file)
            # (1) find and grab from other struct if in this series
            holdit_local_names = [str(xx.name) for xx in structs_list]
            #print("struct.name = %r" % holdit_local_names)
            if str(struct.extends) in holdit_local_names:
                # get index
                ii = holdit_local_names.index(struct.extends)
                # get file contents; yes, this can get us into trouble if we go down too many levels
                [extendsstr,dependencies] = struct_to_rosmsg(structs_list[ii],MDMseriesName,MDMnamespace,rospkgname,structs_list,enums_list,dependencies,level+1)
                if extendsstr == "":
                    print("Trace: %r , name: %s, extends: %s" % (level,struct.name,struct.extends))
                    return ["",dependencies]
                else: # have the info we need, so copy the info into this struct file:
                    s += extendsstr #+ ret
                    extendshandled = 1
            else:
                print("DEBUG: for %s. the struct '%s' was not found in the same file, yet should have been existent!" % (struct.name,struct.extends))
        if extendshandled != 1: # then this must be from some other file
            # (2a) open and read file if in an entirely different series
            #print("*** need to grab this from another file ***")
            if struct.series != None:
                dep = str(struct.series).lower() + "_msgs"
                dep_fixed = find_local_pkg_path(dep)
                if dep_fixed != "":
                    msgfilenamestr = "./" + dep_fixed + "/" + str(struct.extends) + ".msg"
                    extendshandled = 1
                else: # problem with finding directory
                    return ["",dependencies]
            else: # series not given, need to figure out what external file this came from
                [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(struct.extends)
                #ii = struct.extends.find("/")
                if typepre != None:
                    dep = str(typepre).lower() + "_msgs"
                    dep_fixed = find_local_pkg_path(dep)
                    if dep_fixed != "":
                        msgfilenamestr = "./" + dep_fixed + "/" + type_piece + ".msg"
                        extendshandled = 1
                    else: # problem with finding directory
                        return ["",dependencies]
                else: # this should not happen...
                    print("DEBUG: error, name %s 'extends' (%s) should include '/' in this circumstance, failing up..." % (struct.name,struct.extends))
                    return [s,dependencies]
            # now that we have the info we need, copy the info into this struct file:
            msgfile_in = open(msgfilenamestr,'r')
            s += msgfile_in.read()
            msgfile_in.close()
            # (2b) add to pkg msg dependency list for pkg if needed other pkg
            if not(dep in dependencies):
                dependencies.append(dep)
            #print("dependencies for %s %s: %r" % (rospkgname,struct.name,dependencies))
        #if extendshandled != 1: # should never happen
        #    print("We screwed up something here, couldn't include the extends: %s in %s." % (struct.extends,outfile))
        s += "# ----------------------------------------" + ret
    
    for i in range(len(struct.fields)):
        s += "# ----------------------------------------" + ret
        if struct.fields[i].comment != None:
            s += "# " + struct.fields[i].comment.replace("\n","\n# ") + ret
        # trying to get pkg/series to go with type...
        if not hasBasicRosType(struct.fields[i].type):
            enumindex = indexLocalEnumType(struct.fields[i].type,enums_list)
            #print("type '%s', enumindex = %d" % (struct.fields[i].type,enumindex))
            if enumindex != -1: # then need to find and add enum here
                [enumentriesstr,enumtype] = enum_to_rosmsg(enums_list[enumindex])
                [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(struct.fields[i].type)
                s += "# ---" + ret
                s += "# Enumerated type:" + ret
                s += enumentriesstr
                s += "# ---" + ret
                s += enumtype + type_array + " " + struct.fields[i].name
            else:
                if True: #isOtherStructType(struct.fields[i].type,struct) == True: # need to give correct Series before type
                    if (struct.fields[i].series != None) and (struct.fields[i].series != ""): # ...this should be enough, have to give remote series
                        #print("otherstruct in different _msgs")
                        dep = str(struct.fields[i].series).lower() + "_msgs/"
                        s += dep
                        if not(dep in dependencies):
                            dependencies.append(dep)
                    else: # assuming is inside current local set... probably a good assumption
                        #print("otherstruct in same series")
                        #s += rospkgname + "/"
                        s += str(MDMseriesName).lower() + "_msgs/"
                else:
                    print("DEBUG: I don't know what type this is: %s in %s." % (struct.fields[i].type,outfile))
                s += getRosType(struct.fields[i].type) + " " + struct.fields[i].name
        else:
            s += getRosType(struct.fields[i].type) + " " + struct.fields[i].name
        if (struct.fields[i].units != None):
            s += " # " + "units: " + struct.fields[i].units
        if (struct.fields[i].defaultVal != None):
            s += " # " + "default = " + struct.fields[i].defaultVal
        s += ret
    
    out = open(outfile,'w')
    out.write(s)
    out.close()
    
    return [s,dependencies]

def to_rosmsg_comment(comment, indent):
    return str(indent + ('\n' + indent + '--').join(textwrap.wrap('--' + comment))) + '\n'

main()

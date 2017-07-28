#!/usr/bin/python
# Copyright 2017 Government of the United States of America, as represented by the Secretary of the Air Force.
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

"""

*** NOTE THAT THERE IS A BIG PROBLEM HERE CURRENTLY, IN THAT LmcpGen IS
DYNAMIC MESSAGING AND CURRENTLY ROS ONLY SUPPORTS STATIC MESSAGING!! ***

.msg files create + catkin_make okay! example calls:
-- downloaded to ~/github_pulls
as in, 'mkdir -p ~/github_pulls && cd ~/github_pulls && git clone https://github.com/cmcghan/soi_tools.git
then, 'cd ~/github_pulls/soi_tools/lmcp2rosmsg'
then, can call via:
./lmcp2rosmsg.py dir /home/cmcghan/UxAS_cmcghan/OpenUxAS/mdms
-or- (in single-file debug mode, that likely won't create things effectively...)
./lmcp2rosmsg.py file /home/cmcghan/UxAS_cmcghan/OpenUxAS/mdms/CMASI.xml test_output.rosmsg_out
./lmcp2rosmsg.py file /home/cmcghan/UxAS_cmcghan/OpenUxAS/mdms/IMPACT.xml test_output.rosmsg_out
...etc.
-- this is assuming that the 'OpenUxAS' is located under the directory specified as shown above
-- this creates a catkni-like work structure under (in this example) ~/github_pulls/soi_tools/lmcp2rosmsg
as, in, ~/github_pulls/soi_tools/lmcp2rosmsg/catkin_lmcp/src
-- make this an actual workspace via:
'cd ~/github_pulls/soi_tools/lmcp2rosmsg/catkin_lmcp/src; catkin_init_workspace; cd ..; catkin_make'


main() function is the starting point
-- handle_all_files() figures out all the dependencies and then reads in and handles each mdms XML file in order
---- read_XML_MDMs_file_and_output_to_file() is called upon by this (in a for loop) and calls the file read-in stuff to grab the XML content in a usable form (via parse_XML(), storing it in an XMLCollector_rosmsg class) and then calls struct_to_rosmsg() in turn

-- class XMLCollector_rosmsg is the main "class" called to hold all data
---- this depends on the StructInfo, FieldInfo, EnumInfo (that depends in turn on EnumEntry) classes

-- struct_to_rosmsg() is the meat of this file, creating the string that gets written out to each .msg file
---- enums_to_rosmsg() does the same for enumerated types, is a subfunction of struct_to_rosmsg()
"""

import lxml.etree as ET
import argparse as AP
import sys
import textwrap
import os

def get_lxml_to_ros_dict():
    # Call via:
    # lxml_to_ros_dict = get_lxml_to_ros_dict()
    #
    # LXML   -> ROS     (-> Python)
    #-------------------------------
    # bool   -> bool    (-> bool  )
    # string -> string  (-> str   )
    # char   -> string  (-> str   )
    # byte   -> uint8   (-> int   )
    # int64  -> int64   (-> int   )
    # int32  -> int32   (-> int   )
    # uint32 -> uint32  (-> int   )
    # int16  -> int16   (-> int   )
    # uint16 -> uint16  (-> int   )
    # real32 -> float32 (-> float )
    # real64 -> float64 (-> float )

    # LXML   -> ROS     (-> Python)
    #-------------------------------
    # ??     -> int8
    # ??     -> uint64
    # ??     -> time (secs/nsecs uint32)
    # ??     -> duration (secs/nsecs uint32)

    lxml_to_ros_dict = {"bool": "bool", "string": "string", "char": "string",
                        "byte": "uint8", "int64": "int64", "int32": "int32",
                        "uint32": "uint32", "int16": "int16", "uint16": "uint16",
                        "real32": "float32", "real64": "float64"}
    return lxml_to_ros_dict

# see also: http://wiki.ros.org/msg#Building_.msg_Files
def grabPieceArray(type):
    # Call via:
    # [typepre,type_piece,type_array,lxml_to_ros_dict] = grabPieceArray(type)
    #
    
    lxml_to_ros_dict = get_lxml_to_ros_dict()
    
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
    
    return [typepre,type_piece,type_array]

def type_series_fixer(type,series):
    fixedtype = type
    fixedseries = series
    [typepre,type_piece,type_array] = grabPieceArray(type)
    if typepre != "":
        if (series is None) or (series == ""):
            fixedseries = typepre
            fixedtype = type_piece + type_array
            #print("DEBUG: type '%s' -> '%s', series '%s' -> '%s'" % (type,fixedtype,series,fixedseries))
        elif series != typepre: # if gave a series and had series in the typepre but they don't match...
            print("ERROR: mismatch in series given! (type='%s + / + %s', series='%s'." % (typepre,type_piece,series))
            sys.exit(1)
        else: # series and typepre match
            fixedseries = series
            fixedtype = type_piece + type_array
            #print("DEBUG: type '%s' -> '%s', series '%s' -> '%s'" % (type,fixedtype,series,fixedseries))
    return [fixedtype,fixedseries]

def extends_series_fixer(extends,series):
    [fixedextends,fixedseries] = type_series_fixer(extends,series)
    return [fixedextends,fixedseries]

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

    def __str__(self):
        str_ret = self.name
        if self.entries != None:
            str_ret += ' : ' + str(self.entries)
        if self.comment != None:
            str_ret += ' (' + str(self.comment) + ')\n'
        return str_ret

    __repr__ = __str__

class EnumEntry: # new # EnumEntry; basing vars off of LmcpGen/src/avtas/lmcp/lmcpgen/EnumInfo.java
    def __init__(self, name, value, comment):
        self.name = name #public String name = "";
        # enumerated types can be strings (of themselves) if no value explicitly given!
        # see: http://en.cppreference.com/w/cpp/language/enum
        # and for the C++11 way: https://stackoverflow.com/a/24296298
        self.is_Value = False # needed for ROS msg
        self.is_String = False # needed for ROS msg
        if value != None:
            self.value = value # set to value-given # new # public String value = "0";
            self.is_Value = True # will assume is a uint8 here
        else:
            self.value = name # if no value given, set it to the name string (for ROS) # new # public String value = "0";
            self.is_String = True
        self.comment = comment # new #public String comment = "";
        
    def __str__(self):
        str_ret = self.name
        if self.value != None:
            str_ret += ' : ' + str(self.value)
        if self.comment != None:
            str_ret += ' (' + str(self.comment) + ')'
        return str_ret

    __repr__ = __str__

# basing data types off of: LmcpGen/src/avtas/lmcp/lmcpgen/PythonMethods.java
#private static String getPythonType(String type) {
#if (type.toLowerCase().matches("(bool)")) {

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
        
        self.cur_str = None
        self.cur_enum = None; self.cur_entry = None; self.cur_entries = None
        self.cur_field = None; self.cur_fields = None
        self.cur_comment = None
        self.cur_data = None
        
    def start(self, tag, attrib):
        if tag == "MDM" : # new
            if self.cur_comment != None:
                self.MDMcomment = self.cur_comment
                self.cur_comment = None
        if tag == "Struct" : # --> Field
            name = attrib['Name']
            extends = None
            series = None # new
            if 'Extends' in attrib:
                extends = attrib['Extends']
            if 'Series' in attrib: # new
                series = attrib['Series']
            [extendsfix,seriesfix] = extends_series_fixer(extends,series)
            self.cur_fields = []
            self.cur_str = StructInfo(name, extendsfix, seriesfix, self.cur_comment, self.cur_fields)
            self.cur_comment = None # new
        if tag == "Field" :
            name = attrib['Name']
            field_type = attrib['Type']
            series = None
            if 'Series' in attrib: # new
                series = attrib['Series']
            [field_typefix,seriesfix] = type_series_fixer(field_type,series)
            defaultVal = None
            units = None
            if 'Default' in attrib:
                defaultVal = attrib['Default']
            if 'Units' in attrib: # new
                units = attrib['Units']
            self.cur_field = FieldInfo(name, field_typefix, seriesfix, self.cur_comment, defaultVal, units)
            self.cur_comment = None # new
            self.cur_fields.append(self.cur_field)
        if tag == "Enum" : # --> Entry
            name = attrib['Name']
            self.cur_entries = []
            self.cur_enum = EnumInfo(name, self.cur_entries, self.cur_comment)
            self.cur_comment = None # new
        if tag == "Entry":
            name = attrib['Name']
            value = None
            if 'Value' in attrib:
                value = attrib['Value']
            self.cur_entry = EnumEntry(name,value,self.cur_comment)
            self.cur_comment = None
            self.cur_entries.append(self.cur_entry)
        
    def end(self, tag): # don't have data to go with tags until end-tag
        if tag == "SeriesName" : # new
            self.MDMseriesName = strip_whitespace(self.cur_data)
        elif tag == "Namespace" : # new
            self.MDMnamespace = strip_whitespace(self.cur_data)
        elif tag == "Version" : # new
            self.MDMversion = float(strip_whitespace(self.cur_data))
        elif tag == "Struct" : # --> Field
            self.cur_str.fields = self.cur_fields
            self.structs.append(self.cur_str)
        #elif tag == "Field" :
            #pass
        elif tag == "Enum" : # --> Entry
            self.cur_enum.entries = self.cur_entries
            self.enums.append(self.cur_enum)
        #elif tag == "Entry":
            #pass
        self.cur_comment = None
        self.cur_data = None # new
        
    def data(self, data): # new # see: http://lxml.de/parsing.html#the-target-parser-interface
        self.cur_data = data
        #print("Data: %r" % data) # note: this will give back unicode strings

    def comment(self, text): # new # do NOT set a var in __init__ to same name with =None, will wipe out f(n)!!
        self.cur_comment = strip_whitespace(text) # this turns unicode string to python string
        
    def close(self):
        return

def hasBasicRosType(type):
    [typepre,type_piece,type_array] = grabPieceArray(type)
    lxml_to_ros_dict = get_lxml_to_ros_dict()
    
    if (type_piece.lower() in lxml_to_ros_dict):
        return True
    else:
        return False

def getRosType(type): # type assumed to be a string
    # ROS uses same array [] or [#] as LXML, so could just find-replace portions of the string as-needed...
    # except that this won't work if part of a created type includes these
    [typepre,type_piece,type_array] = grabPieceArray(type)
    lxml_to_ros_dict = get_lxml_to_ros_dict()
    
    if (type_piece.lower() in lxml_to_ros_dict):
        holdstr = lxml_to_ros_dict[type_piece.lower()]
    else:
        holdstr = type_piece

    if typepre !=  '':
        return typepre+"_msgs/"+holdstr+type_array
    else:
        return holdstr+type_array

#indexLocalEnumType(struct.fields[i].type,enums)
def indexLocalEnumType(type,enums_list):
    index = -1
    # find if enum if in this series
    holdit_enum_names = [str(xx.name) for xx in enums_list]
    [typepre,type_piece,type_array] = grabPieceArray(type)
    if str(type_piece) in holdit_enum_names: # then get index
        index = holdit_enum_names.index(str(type_piece))
    return index

#isOtherStructType(struct.fields[i].type,struct)
#def isOtherStructType(type,struct):
#    boolAns = False
#    
#    return boolAns

# get_struct_deps() is modified from struct_to_rosmsg (removes unnecessary additional computation)
def get_struct_deps(struct,MDMseriesName,MDMnamespace,enums_list,dependencies=[],extendersby={}):
    # struct.{name,extends,series,comment,fields}
    # fields.{name,comment,type,defaultVal,units}
    #
    # extendersby is used to add info to comments in the .msg files for each struct and fieldtype that is listed in the dictionary / allows for downcasting!
    
    
    str_rosseries_and_name = str(str(MDMseriesName).lower() + '_msgs/' + struct.name)
    if not (str_rosseries_and_name in extendersby.keys()): # if not in dict then create new key for it with blank entry
        extendersby[str_rosseries_and_name] = [ ] # (that way, every struct gets an entry :)
    
    # get dependencies in struct if has an extends
    if struct.extends != None:
        #
        # now, need to find and add all parts of other struct (we are 'extending' from) here:
        #
        # find other struct
        if struct.series == None or str(struct.series) == "" or str(struct.series).lower() == str(MDMseriesName).lower(): # then in this series (same MDM file)
            str_rosseries_and_name = str(str(MDMseriesName).lower() + '_msgs/' + struct.name)
            str_extendsseries_and_name = str(str(MDMseriesName).lower() + '_msgs/' + struct.extends)
            if not (str_extendsseries_and_name in extendersby.keys()): # if not in dict then create new key and add to the list
                extendersby[str_extendsseries_and_name] = [ str_rosseries_and_name ]
            else: # if in dict
                if str_rosseries_and_name not in extendersby[str_extendsseries_and_name]: # if not already in the key's value list
                    holdlist = extendersby[str_extendsseries_and_name] # add to value list
                    holdlist.append( str_rosseries_and_name )
                    #extendersby[str_extendsseries_and_name] = holdlist
                    extendersby.update( {str_extendsseries_and_name: holdlist} )
                else:
                    pass # already in there, nothing to do (should never get here unless there's some bad/broken dual-naming thing going on)

            # (1) find and grab from other struct if in this series
            # other struct will show an external dependency when we get to it, will be added then
            # we only really care about global pkg deps per-XML MDMs file
            pass
        else: # then this must be from some other file
            # (2a) open and read file if in an entirely different series
            #print("*** need to grab this from another file ***")
            # in get_struct_deps, don't need to actually open that file currently, just save what it's looking for...
            dep = str(struct.series).lower() + "_msgs"

            # (2b) add to pkg msg dependency list for pkg if needed other pkg
            if not(dep in dependencies):
                dependencies.append(dep)
            #print("dependencies for %s %s: %r" % (rospkgname,struct.name,dependencies))

            str_rosseries_and_name = str(str(MDMseriesName).lower() + '_msgs/' + struct.name)
            str_extendsseries_and_name = str(dep + '/' + struct.extends)
            if not (str_extendsseries_and_name in extendersby.keys()): # if not in dict then create new key and add to the list
                extendersby[str_extendsseries_and_name] = [ str_rosseries_and_name ]
            else: # if in dict
                if not (str_rosseries_and_name in extendersby[str_extendsseries_and_name]): # if not already in the key's value list
                    holdlist = extendersby[str_extendsseries_and_name] # add to value list
                    holdlist.append( str_rosseries_and_name )
                    #extendersby[str_extendsseries_and_name] = holdlist
                    extendersby.update( {str_extendsseries_and_name: holdlist} )
                else:
                    pass # already in there, nothing to do (should never get here unless there's some bad/broken dual-naming thing going on)
    else: # doesn't extend anything
        pass

    # get dependencies in fields
    for i in range(len(struct.fields)):
        # trying to get pkg/series to go with type...
        if not hasBasicRosType(struct.fields[i].type):
            enumindex = indexLocalEnumType(struct.fields[i].type,enums_list)
            if enumindex != -1: # then need to find and add enum here
                pass # enums are never external
            else:
                # need to give correct Series before type
                if (struct.fields[i].series != None) and (struct.fields[i].series != "") and str(struct.fields[i].series).lower() != str(MDMseriesName).lower(): # then in this series (same MDM file)
                    #print("otherstruct in different _msgs")
                    dep = str(struct.fields[i].series).lower() + "_msgs"
                    if not(dep in dependencies):
                        dependencies.append(dep)
                else: # assuming is inside current local set... probably a good assumption
                    pass
        else:
            pass

    return [dependencies,extendersby] # None is returned if there is a problem, [] is returned if no external deps

def get_all_xml_file_deps(MDMseriesName,MDMnamespace,structs_list,enums_list,extendersby={}):
    dependencies = []
    for s in structs_list:
        [dependencies,extendersby] = get_struct_deps(s,MDMseriesName,MDMnamespace,enums_list,dependencies,extendersby)
        if dependencies is None:
            print("DEBUG: problem with finding dependencies for %s!"  % s.name)
            return None
    return [dependencies,extendersby]

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
    extendersby = {}
    for filename in matches_found:
        c = parse_XML(filename) # c as in collector
        [dependencies,extendersby] = get_all_xml_file_deps(c.MDMseriesName,c.MDMnamespace,c.structs,c.enums,extendersby)
        if dependencies == None:
            sys.exit(0)
        unordered_list.append([filename,str(c.MDMseriesName).lower() + "_msgs",dependencies])
    
    # dependencies and extendersby should both be completely filled in, now
    print("DEBUG: extendersby = %r" % extendersby)
    
    # then get them 'sorted' in the order in which dependencies need to be handled
    i = 0
    sorted_list = [] # entries are: [filename,str(c.MDMseriesName).lower() + "_msgs",dependencies]
    deps_added = []
    requires_sorting = []
    for i in range(len(unordered_list)):
        if unordered_list[i][2] == []: # if dependencies == []
            sorted_list.append(unordered_list[i])
            deps_added.append(unordered_list[i][1])
        else: # add to requires-sorting part
            requires_sorting.append(unordered_list[i])
    #print("sorted_list: %r" % sorted_list)
    #print("deps_added: %r" % deps_added)
    #print("requires_sorting: %r" % requires_sorting)
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
    #print("requires_sorting: %r" % requires_sorting)
    #print("sorted_list: %r" % sorted_list)
    
    # now that everything's sorted in an order that'll  be able to handle all deps, get each file handled!
    
    pkg_structdata_dict = {}
    # start by reading in all info from files, so that if we need to handle stuff, we can...
    for i in range(len(sorted_list)):
        in_file = sorted_list[i][0]
        collector = None
        collector = parse_XML(in_file)
        pkg_structdata_dict.update({str(sorted_list[i][1]): collector}) # {'name_of_pkg_msgs' : collector_struct}
    
    # now that everything's sorted in an order that'll  be able to handle all deps, get each file handled!
    for i in range(len(sorted_list)):
        #print("---\nDealing with %s..." % sorted_list[i][0])
        in_file = sorted_list[i][0]
        read_XML_MDMs_file_and_output_to_file(in_file,None,pkg_structdata_dict,extendersby) # out_file = None
        #print("Done dealing with %s...\n---" % sorted_list[i][0])
    # after this, should be done!

def main():
    #parse all of the arguments
    ap = AP.ArgumentParser(description='Converts an LMCP xml file into AADL datatypes')
    ap.add_argument('runtype', metavar='runoption', type=str, choices=["file","dir"], help='the option "dir" or "file" that decides the run (dir for directory as input_file)')
    ap.add_argument('input_file', metavar='input', type=str, help='the input LMCP xml file if runtype is "file", the directory holding the LMCP xml files if runtype is "dir" (e.g, "/home/username/UxAS_pulls/OpenUxAS/mdms")')
    ap.add_argument('output_file', metavar='output', type=str, nargs='?', default=None, help='the output AADL file')

    args = ap.parse_args()
    args = vars(args)

    runtype = args.get('runtype')
    in_file = args.get('input_file')
    out_file = args.get('output_file')
    
    if in_file is None:
        print("Input filename not given. Exiting.")
        sys.exit(1)
    if runtype != "dir" and out_file is None:
        print("Output filename not given. Exiting.")
        sys.exit(1)
    if in_file == out_file:
        print('the input file and output file must be different names')
        sys.exit(1)

    if runtype == "dir": # chaining multiple things together
        print("Getting 'initial' ROS .msg stuff handled {'datatype': , 'datastring': }...")
        UxASxmlSeries = "UXAS_REQUIRED"
        UxASxmlStruct = "DatatypeDatastring"
        UxASxmlROSpkgType = UxASxmlSeries.lower()+ '_msgs/' + UxASxmlStruct
        UxASxmlString = """<?xml version="1.0" encoding="UTF-8"?>
                           <!DOCTYPE MDM SYSTEM 'MDM.DTD'>
                           
                           <!--
                           # $ROSPKG+TYPE$ is a...
                           # Basic structure for all typed data coming from UxAS as JSON-dict converted LmcpGen messages
                           #string datatype
                           #string datastring
                           -->
                           <MDM>
                               <SeriesName>$SERIES$</SeriesName>
                               <Namespace>ros/uxasjson</Namespace>
                               <Version>3</Version>
                               <StructList>

                                   <!-- Base class for all straight-send JSON-dict messages from UxAS currently -->
                                   <Struct Name="$STRUCT$">
                                       <!-- A string containing the Series/Struct name (type / format of object) of the UxAS data being sent -->
                                       <Field Name="datatype" Type="string"/>
                                       <!-- A string containing the dictionary-format of UxAS object values for the datatype -->
                                       <Field Name="datastring" Type="string"/>
                                   </Struct>
                                
                               </StructList>
                           </MDM>"""
        UxASxmlString = UxASxmlString.replace("$SERIES$", UxASxmlSeries)
        UxASxmlString = UxASxmlString.replace("$STRUCT$", UxASxmlStruct)
        UxASxmlString = UxASxmlString.replace("$ROSPKG+TYPE$", UxASxmlROSpkgType)
        outfileToWriteName = UxASxmlSeries + ".xml"
        outfile_toWrite = open(outfileToWriteName,'w')
        outfile_toWrite.write(UxASxmlString)
        outfile_toWrite.close()
        read_XML_MDMs_file_and_output_to_file(outfileToWriteName,out_file=None,pkg_structdata_dict=None,extendersby={UxASxmlROSpkgType: []})
        print("running over entire directory of xml files...")
        handle_all_files(in_file) # (xmldirstr)
    elif runtype == "file": # old/original way to do it
        print("running over single xml file...")
        read_XML_MDMs_file_and_output_to_file(in_file,out_file)
    else:
        print("ERROR: Unknown input switch, exiting.")
        sys.exit(1)

def parse_XML(in_file):
    contents = open(in_file, 'r').read()
    collector = XMLCollector_rosmsg()
    parser = ET.XMLParser(target = collector)
    result = ET.XML(contents, parser) # changes the parser(=collector) internally
    return collector

# ref: https://en.wikipedia.org/wiki/Code_injection#Shell_injection
# could use Python 3's shlex.quote(s) instead, see: https://docs.python.org/3/library/shlex.html#shlex.quote
# and: https://stackoverflow.com/questions/34898430/escaping-for-proper-shell-injection-prevention
import string
def found_unsafe_inputs_in_str(str_to_check):
    status = -1
    # look for what shouldn't be there:
    # from: https://hg.python.org/cpython/file/63354d11a741/Lib/shlex.py#l278
    #_--> find_unsafe = re.compile(r'[^\w@%+=:,./-]', re.ASCII).search
    # flag if find any of: [^\w@%+=:,./-]
    # from Wikipedia: https://en.wikipedia.org/wiki/Code_injection#Shell_injection
    # flag if find any of: ;|`$&><
    #for ch in [";","|","`","$","&",">","<"."'",'"',"[","^","\\","w","@","%","+","=",":",",",".","/","-","]"]:
    for ch in [";","|","`","$","&",">","<","'",'"',"[","^","\\","w","@","%","+","=",":",",","-","]"]:
        if str_to_check.find(ch) != -1:
            status = 1
            print("DEBUG: found char '%s'" % ch)
            return 1
    # make sure what is there is only in the allowed set:
    for ch in str_to_check:
        if not(ch in str(string.letters + string.whitespace + string.digits + "_./")): # allow underscore in names and . / for directory parsing
            status = 1
            print("DEBUG: unsafe char '%s'" % ch)
            return 1
    return status


def read_XML_MDMs_file_and_output_to_file(in_file,out_file=None,pkg_structdata_dict=None,extendersby=None): # *** TODO: if extendersby is not None, add info to comments in the file for each struct and fieldtype that is listed in the extendersby dictionary!!! ***
    if in_file is None:
        print("Input filename not given. Exiting.")
        sys.exit(1)
    if out_file is not None:
        print("Output filename is given. Will also write to given file, not just individual files.")
    if in_file == out_file:
        print('The input file and output file must be different names')
        sys.exit(1)

    collector = parse_XML(in_file) # read into pkg_structdata_dict before, but we don't know the _msg pkg we're working on yet, just the filename to read
    
    # *** FOR DEBUGGING: ***
    #print("seriesName = %r" % collector.MDMseriesName)
    #print("namespace = %r" % collector.MDMnamespace)
    #print("version = %r" % collector.MDMversion)
    #print("comment = %r" % collector.MDMcomment)
    #print("structs = %r" % collector.structs)
    #print("enums = %r" % collector.enums)
    
    # package name is detemrined by collector.MDMseriesName, collector.MDMnamespace
    # output filenames for each struct is determined from collector.structs.name, collector.structs.series
    rospkgname = str(collector.MDMnamespace).lower() + "/" + str(collector.MDMseriesName).lower() + "_msgs"
    #rospkgname = str(collector.seriesName).lower() + "_msgs" # this won't handle the "namespaces" well
    
    # might want to look into using subprocess.call() instead of os.system(), as per:
    # -- https://stackoverflow.com/questions/89228/calling-an-external-command-in-python
    # -- https://docs.python.org/2/library/subprocess.html
    
    # given catkin workspace directory (run at same level as script?)
    catkinws_dir = 'catkin_lmcp' # may get this from "out_file" later instead of using it as a debug output of (last) XMl file written to disk, but for now use this
    
    # remove build and devel dirs from the catkin workspace directory: # runs multiple times in this f(n)...
    if found_unsafe_inputs_in_str(str(catkinws_dir)) == -1:
        print("removing 'build' and 'devel' directories from catkin workspace '%s'..." % catkinws_dir)
        os.system('rm -rf %s/build' % str(catkinws_dir))
        os.system('rm -rf %s/devel' % str(catkinws_dir))
    pre_dir = str(catkinws_dir) + "/src/"
    rospkgname = pre_dir + rospkgname # modify this to be a subdir of the catkin_workspace
    
    print("------------------\n*** Starting package creation of %s ***" % rospkgname)
    
    # creating subdirectory 'X_msgs/msg'...
    #os.system('mkdir -p %s' % str(rospkgname))
    if found_unsafe_inputs_in_str(str(rospkgname)) == -1:
        print("creating subdirectory '%s/msg'..." % str(rospkgname))
        os.system('mkdir -p %s/msg' % str(rospkgname)) # for correct ROS pkg structure
    else:
        print("Unsafe rospkgname string '%s' created from file=%s. Exiting before run." % (str(rospkgname),in_file))
        sys.exit(1)
    
    # make CMakeLists.txt and package.xml that is correct for the catkin pkg
    [dependencies,holdsingle_extendersby] = get_all_xml_file_deps(collector.MDMseriesName,collector.MDMnamespace,collector.structs,collector.enums)#,{})
    pkgmsgstr = str(str(collector.MDMseriesName).lower()+"_msgs")
    pkgdepsstr = " ".join(dependencies)
    pkgdepstofilestr = "\n  ".join(dependencies)
    if found_unsafe_inputs_in_str(str(rospkgname)) == -1 and found_unsafe_inputs_in_str(pkgmsgstr) == -1 and found_unsafe_inputs_in_str(pkgdepsstr) == -1:
        # if the directory already exists, remove it, then create the package again anew (so that we have a clean run)
        if os.path.isdir(rospkgname): # if (full)path is a directory that already exists...
            if (" " in rospkgname) or (" / " in rospkgname) or ("/ " in rospkgname) or (".." in rospkgname) or (rospkgname == "/"):
                # warning! unsafe! don't do the rm command on blatantly root-level system directory or something that might be (already) bumping up up extra levels!
                print("Warning, rospkgname='%s', not going through with the rm command!")
                sys.exit(1)
            else: # should be (relatively) safe to remove the directory
                print("removing path: '%s'" % str(rospkgname))
                os.system('rm -rf ./%s' % str(rospkgname)) # remove the directory
        
        # create the ROS package
        #os.system('cd %s/.. && catkin_create_pkg %s %s' % (str(rospkgname),pkgmsgstr,pkgdepsstr))
        # easier to modify the CMakeLLists.txt and package.xml without giving it package dependency inputs that change the file
        print("creating package '%s' in directory '%s'..." %(str(rospkgname),pkgmsgstr))
        os.system('cd ./%s/.. && catkin_create_pkg %s' % (str(rospkgname),pkgmsgstr))
        # then we need to modify the two files:

        # (re-)creating subdirectory 'X_msgs/msg'...
        #os.system('mkdir -p %s' % str(rospkgname))
        if found_unsafe_inputs_in_str(str(rospkgname)) == -1:
            print("creating subdirectory '%s/msg'..." % str(rospkgname))
            os.system('mkdir -p %s/msg' % str(rospkgname)) # for correct ROS pkg structure
        else:
            print("Unsafe rospkgname string '%s' created from file=%s. Exiting before run." % (str(rospkgname),in_file))
            sys.exit(1)
        
        #-- fixing CMakeLists.txt
        
        oldstrtofind = []
        newstrtoreplacewith = []
        
        oldstrtofind.append("find_package(catkin REQUIRED)")
        newstrtoreplacewith.append("find_package(catkin REQUIRED COMPONENTS\n  roscpp\n  rospy\n  #rosjava # not necessary to include rosjava, just install ros-kinetic-rosjava and it will automatically run genjava\n  std_msgs\n  %s\n  message_generation\n)" % pkgdepstofilestr)

        holdstrstructnames = ""
        for s in collector.structs:
            holdstrstructnames += "  " + s.name + ".msg\n"
        oldstrtofind.append("# add_message_files(\n#   FILES\n#   Message1.msg\n#   Message2.msg\n# )")
        newstrtoreplacewith.append("add_message_files(\n  FILES\n%s)" % holdstrstructnames) # '%s' instead of '  %s\n' because already has preceding '  ' and trailing '\n'
        
        oldstrtofind.append("# generate_messages(\n#   DEPENDENCIES\n#   std_msgs  # Or other packages containing msgs\n# )")
        newstrtoreplacewith.append("generate_messages(\n  DEPENDENCIES\n  std_msgs  # Or other packages containing msgs\n  %s\n)" % pkgdepstofilestr)
        
        holdstrstructnamespluspre = ""
        for dep in dependencies:
            holdstrstructnamespluspre += "  CATKIN_DEPENDS " + dep + "\n"
        oldstrtofind.append("catkin_package(\n#  INCLUDE_DIRS include\n#  LIBRARIES %s\n#  CATKIN_DEPENDS other_catkin_pkg\n#  DEPENDS system_lib\n)" % pkgmsgstr)
        newstrtoreplacewith.append("catkin_package(\n#  INCLUDE_DIRS include\n#  LIBRARIES %s\n  CATKIN_DEPENDS message_runtime\n%s#  DEPENDS system_lib\n)" % (pkgmsgstr,holdstrstructnamespluspre)) # '%s' instead of '  %s\n' because already has preceding '  ' and trailing '\n'

        #print("./" + str(rospkgname) + "/CMakeLists.txt")
        CMakeLists_in = open("./" + str(rospkgname) + "/CMakeLists.txt","r")
        filedata = CMakeLists_in.read()
        CMakeLists_in.close()
        #print("\n-----before------\n" + filedata + "\n----------------\n")
        fixedfiledata_str = filedata
        for i in range(len(oldstrtofind)): # now, perform find-replace ops on the CMakeLists.txt file!
            fixedfiledata_str = fixedfiledata_str.replace(oldstrtofind[i],newstrtoreplacewith[i])
        #print("\n-----after------\n" + fixedfiledata_str + "\n----------------\n")
        CMakefile_out = open("./" + str(rospkgname) + "/CMakeLists.txt", 'w')
        CMakefile_out.write(fixedfiledata_str)
        CMakefile_out.close()
        print("Finished modification of file: CMakeLists.txt")
        
        #-- fixing package.xml
        
        oldstrtofind = []
        newstrtoreplacewith = []

        holdstrbuildpkgdeps = ""
        holdsrunpkgdeps = ""
        for dep in dependencies:
            holdstrbuildpkgdeps += "  <build_depend>" + dep + "</build_depend>\n"
            holdsrunpkgdeps += "  <run_depend>" + dep + "</run_depend>\n"
        oldstrtofind.append("  <buildtool_depend>catkin</buildtool_depend>\n")
        newstrtoreplacewith.append("  <buildtool_depend>catkin</buildtool_depend>\n  <build_depend>message_generation</build_depend>\n  <build_depend>std_msgs</build_depend>\n%s  <run_depend>message_runtime</run_depend>\n  <run_depend>std_msgs</run_depend>\n%s" % (holdstrbuildpkgdeps,holdsrunpkgdeps))

        #print("./" + str(rospkgname) + "/package.xml")
        packagexml_in = open("./" + str(rospkgname) + "/package.xml","r")
        filedata = packagexml_in.read()
        packagexml_in.close()
        #print("\n-----before------\n" + filedata + "\n----------------\n")
        fixedfiledata_str = filedata
        for i in range(len(oldstrtofind)): # now, perform find-replace ops on the package.xml file!
            fixedfiledata_str = fixedfiledata_str.replace(oldstrtofind[i],newstrtoreplacewith[i])
        #print("\n-----after------\n" + fixedfiledata_str + "\n----------------\n")
        packagexml_out = open("./" + str(rospkgname) + "/package.xml", 'w')
        packagexml_out.write(fixedfiledata_str)
        packagexml_out.close()
        print("Finished modification of file: package.xml")
        
    else:
        print("Unsafe MDMnamespace string '%s' or unsafe dependencies '%s' or directory '%s' created from file=%s. Exiting before run." % (pkgmsgstr,pkgdepsstr,rospkgname,in_file))
        sys.exit(1)
    
    print("Beginning .msg file(s) creation for %s_msg..." % str(collector.MDMseriesName).lower())
    
    # use collector.MDMcomment for short README.txt document in repo:
    readme_str = "##Series Name\n" + str(collector.MDMseriesName) + '\n'
    #readme_str += "##Namespace\n" + rospkgname + '\n'
    readme_str += "##Namespace\n" + str(collector.MDMnamespace) + '\n'
    readme_str += "##Version\n" + str(collector.MDMversion) + '\n'
    readme_str += "<hr>\n"
    readme_str += str(collector.MDMcomment) + '\n'
    out = open("./" + rospkgname + "/README.md", 'w')
    out.write(readme_str)
    out.close()
    
    # get the .msg file content made (string structstr) and then written to disk (at out_file)
    if out_file is not None:
        out = open(out_file,'w')
    # output filenames for each struct is determined from collector.structs.name, collector.structs.series
    for s in collector.structs: # struct_to_rosmsg will internally write to individual files, but below line...
        [structstr,specificdependencies] = struct_to_rosmsg(s,collector.MDMseriesName,collector.MDMnamespace,rospkgname,pre_dir,collector.structs,collector.enums,pkg_structdata_dict,[],extendersby) # ...we don't give it the package "dependencies" here -- struct_to_rosmsg() is internally calculating specific per-struct dependencies
        if out_file is not None:
            out.write(structstr) # ...(also) writes to full file
            out.write("\n")
            out.write("# ----------------------------------------\n")
            out.write("# ----------------------------------------\n")
            out.write("# ----------------------------------------\n")
            out.write("\n")
        #if specificdependencies != []:
        #    print("%s/%s has external dependencies: %r" % (rospkgname,str(s.name),specificdependencies))
    
    if out_file is not None:
        out.close()
        
    print(".msg file(s) creation for %s_msg completed!" % str(collector.MDMseriesName).lower())

# see also: http://answers.ros.org/question/9427/enum-in-msg/
# and: http://wiki.ros.org/msg#Constants
def enum_to_rosmsg_piece(enum):
    #
    # note: we are going to have to handle all the enums first, because
    # they need to be written into the corresponding .msg files at-need
    #
    ret = '\n'
    
    s_linelist = []
    
    # EnumInfo
    # {name , comment , entries}
    #s = "# Enum: " + enum.name + ret
    s_linelist.append("# Enum: " + enum.name)
    #print("Enum name = %s" % enum.name)
    if enum.comment != None:
        #s += "# " + enum.comment.replace("\n","\n# ") + ret
        s_linelist.append("# " + enum.comment.replace("\n","\n# "))
    for entry in enum.entries:
        shold = ""
        # EnumEntry
        #{name , is_Value , is_String , value , comment}
        if entry.comment != None:
            #s += "# " + entry.comment.replace("\n","\n# ") + ret
            s_linelist.append("# " + entry.comment.replace("\n","\n# "))
        if entry.is_Value:
            #s += "uint8 "
            shold += "uint8 "
            enumtype="uint8" # should be consistent for all entries
        elif entry.is_String:
            #s += "string "
            shold += "string "
            enumtype="string" # should be consistent for all entries
        else:
            #s += "# ERROR: (--None--) "
            shold += "# ERROR: (--None--) "
            print("DEBUG: unknown enum entry type %s in enum %s" % (entry.name,enum.name))
        #s += entry.name + "=" + entry.value + ret
        shold += entry.name + "=" + entry.value
        s_linelist.append(shold)
    #print("enum s = %s" % s)
    #return [s,enumtype]
    return [s_linelist,enumtype]

def find_local_pkg_path(dirname_to_find,dir_to_look_in='./'):
    # reference: https://stackoverflow.com/a/2186565
    import fnmatch
    import os
    matches_found = []
    for localroot, dirnames, filenames in os.walk(dir_to_look_in):
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

def get_extendsby_strings_for_rosmsg_piece(extendersby,seriesName,typename): # seriesName,typename is expected to be either MDMseriesName,struct.name or struct.fields[i].series,struct.fields[i].type
    exby_str = ""
    ret = "\n"
    
    # just in case, get rid of potential array piece
    [typepre,type_piece,type_array] = grabPieceArray(typename) # struct.fields[i].type)
    typename = type_piece
    
    if extendersby != None:
        str_rosseries_and_name = str(str(seriesName).lower() + '_msgs/' + typename)
        if not (str_rosseries_and_name in extendersby.keys()):
            print("DEBUG: issue, every struct key should have a pair!! %r" % str_rosseries_and_name)
        else:
            if len(extendersby[str_rosseries_and_name]) == 0: # nothing uses this in an 'extend'
                exby_str += "# Extended by: (n/a, no children)" + ret
            else: # write out all things extending this / extended by...
                # set up the list of things and go through figuring out what need to add?
                # -- no, this is unnecessary, there should be no loops
                holdlist = extendersby[str_rosseries_and_name]
                exby_str += "# Extended by: " + holdlist[0]
                i = 1
                while i < len(holdlist):
                    exby_str += ", " + holdlist[i]
                    i = i+1
                exby_str += ret
                
                childrenlist = []
                while len(holdlist) > 0:
                    single_extender = holdlist[0]
                    if not (single_extender in childrenlist):
                        childrenlist.append([single_extender, extendersby[single_extender] ])
                    holdlist = holdlist[1:len(holdlist)]
                    holdlist = holdlist + extendersby[single_extender] # add more entries to back of list, [] will change nothing (no error)
                # print out all sub-children and sub-subchildren, etc.
                for j in range(len(childrenlist)):
                    if len(childrenlist[j][1]) == 0: # nothing uses this in an 'extend'
                        exby_str += "# Child " + childrenlist[j][0] + " extended by: (n/a, no children)" + ret
                    else: #if len(childrenlist[j][1]) > 1: # then this has children of its own to print out
                        exby_str += "# Child " + childrenlist[j][0] + " extended by: " + childrenlist[j][1][0]
                        i = 1
                        while i < len(childrenlist[j][1]):
                            exby_str += ", " + childrenlist[j][1][i]
                            i = i+1
                        exby_str += ret
    return exby_str

# see also: http://docs.ros.org/kinetic/api/calibration_msgs/html/msg/JointStateCalibrationPattern.html
def struct_to_rosmsg(struct,MDMseriesName,MDMnamespace,rospkgname,pre_dir,structs_list,enums_list,pkg_structdata_dict=None,dependencies=[],extendersby=None,level=0):
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
    # -> 'Field Default' of "MSL" is the initial value to set this to... except that ROS doesn't support this
    #    (...does 0mq??)
    #
    
    # struct.{name,extends,series,comment,fields}
    # fields.{name,comment,type,defaultVal,units}
    
    if (level>100):
        print("It looks like we went down the rabbit hole on 'extends'.")
        return ""
    
    ret = "\n"
    
    #outfile = "./" + rospkgname + "/" + struct.name + ".msg"
    outfile = "./" + rospkgname + "/msg/" + str(struct.name) + ".msg" # for correct ROS pkg structure
    
    s = "# Struct: " + struct.name + ret
    if struct.comment != None:
        s += "# " + struct.comment.replace("\n","\n# ") + ret
    
    s += get_extendsby_strings_for_rosmsg_piece(extendersby,MDMseriesName,struct.name) # "struct" slot here only needs ".name" part
    
    if struct.extends is None:
        s += "# Extends: (n/a, no parents)" + ret
    else: #if struct.extends != None:
        #s += ret
        if struct.series != None:
            s += "# Extends: " + str(struct.series).lower() + "_msgs/" + struct.extends + ret
        else:
            s += "# Extends: " + str(MDMseriesName).lower() + "_msgs/" + struct.extends + ret
        s += "# ----------------------------------------" + ret
        s += "# ----------------------------------------" + ret
        
        #
        # now, need to find and add all parts of other struct (we are 'extending' from) here:
        #
        # find other struct -- note: because we are adding "extends" first, we shouldn't be "doubling up" on things like enums below (enums below will check this piece since it's already in the string s
        if struct.series == None or str(struct.series) == "" or str(struct.series).lower() == str(MDMseriesName).lower(): # then in this series (same MDM file)
            # (1) find and grab from other struct if in this series
            holdit_local_names = [str(xx.name) for xx in structs_list]
            #print("struct.name = %r" % holdit_local_names)
            if str(struct.extends) in holdit_local_names:
                # get index
                ii = holdit_local_names.index(struct.extends)
                # get file contents; yes, this can get us into trouble if we go down too many levels
                [extendsstr,dependencies] = struct_to_rosmsg(structs_list[ii],MDMseriesName,MDMnamespace,rospkgname,pre_dir,structs_list,enums_list,pkg_structdata_dict,dependencies,extendersby,level+1)
                if extendsstr == "":
                    print("Trace: %r , name: %s, extends: %s" % (level,struct.name,struct.extends))
                    return ["",dependencies]
                else: # have the info we need, so copy the info into this struct file:
                    s += extendsstr #+ ret
            else:
                print("DEBUG: for %s. the struct '%s' was not found in the same file, yet should have been existent!" % (struct.name,struct.extends))
                return ["",dependencies]
        else: # then this must be from some other file
            # (2a) open and read file if in an entirely different series
            #print("*** need to grab this from another file ***")
            dep = str(struct.series).lower() + "_msgs"
            dep_fixed = find_local_pkg_path(dep,pre_dir) # look from the correct prefix dir where the files are being created
            if dep_fixed != "":
                #msgfilenamestr = "./" + dep_fixed + "/" + str(struct.extends) + ".msg"
                msgfilenamestr = "./" + dep_fixed + "/msg/" + str(struct.extends) + ".msg" # for correct ROS pkg structure
                extendshandled = 1
            else: # problem with finding directory
                print("Problem is with struct %s.\n--" % struct.name)
                return ["",dependencies]
            # now that we have the info we need, copy the info into this struct file:
            msgfile_in = open(msgfilenamestr,'r')
            s += msgfile_in.read()
            msgfile_in.close()
            # (2b) add to pkg msg dependency list for pkg if needed other pkg
            if not(dep in dependencies):
                dependencies.append(dep)
            #print("dependencies for %s %s: %r" % (rospkgname,struct.name,dependencies))
        s += "# ----------------------------------------" + ret
    
    for i in range(len(struct.fields)):
        s += "# ----------------------------------------" + ret
        if struct.fields[i].comment != None:
            s += "# " + struct.fields[i].comment.replace("\n","\n# ") + ret
        # trying to get pkg/series to go with type...
        if not hasBasicRosType(struct.fields[i].type):
            enumindex = indexLocalEnumType(struct.fields[i].type,enums_list)
            seriesdep_enumindex = -1
            pkgname_lookingfor = ""
            if (enumindex == -1): # no point in looking further if found it locally
                if struct.fields[i].series == None or struct.fields[i].series == "":
                    pkgname_lookingfor = str(MDMseriesName).lower()+'_msgs'
                else:
                    pkgname_lookingfor = str(struct.fields[i].series).lower()+'_msgs'
                seriesdep_enumslist = pkg_structdata_dict[pkgname_lookingfor].enums
                seriesdep_enumindex = indexLocalEnumType(struct.fields[i].type,seriesdep_enumslist)
            #if (str(struct.name) == 'LoiterAction'):
            #    print("*** DEBUG *** struct.fields[i].type = %s, struct.fields[i].series = %s , pkgname_lookingfor = %s , enumindex = %d , seriesdep_enumindex = %d" % (struct.fields[i].type,struct.fields[i].series,pkgname_lookingfor,enumindex,seriesdep_enumindex))
            #print("type '%s', enumindex = %d" % (struct.fields[i].type,enumindex))
            if enumindex != -1 or seriesdep_enumindex != -1: # then need to grab and add enum here
                if enumindex != -1:
                    [enumentriesstr_linelist,enumtype] = enum_to_rosmsg_piece(enums_list[enumindex])
                if seriesdep_enumindex != -1:
                    [enumentriesstr_linelist,enumtype] = enum_to_rosmsg_piece(seriesdep_enumslist[seriesdep_enumindex])
                [typepre,type_piece,type_array] = grabPieceArray(struct.fields[i].type)
                s += "# ---" + ret
                s += "# Enumerated type:" + ret
                #s += enumentriesstr
                if s.find("\n".join(enumentriesstr_linelist)) == True: # then we already added this enum to this .msg file, don't add it again
                    pass
                else: # need to check line-by-line to make sure this hasn't been added before
                    for singleline in enumentriesstr_linelist:
                        if s.find(str(singleline)) != -1: # then we already added this line to this .msg file, don't add it again
                            s += "# " + singleline + ret # not leaving it out entirely, commenting it out!!
                            #if (str(struct.name) == 'LoiterAction'):
                            #    print("DEBUG: Commented '%s'" % singleline)
                        else:
                            s += singleline + ret
                            #if (str(struct.name) == 'LoiterAction'):
                            #    print("DEBUG: Added '%s'" % singleline)
                s += "# ---" + ret
                s += enumtype + type_array + " " + struct.fields[i].name
            else: # then need to list the 'correct_series_pkg_msgs/datatype' here
                
                # first print all deps for the fieldseries/field
                if (struct.fields[i].series != None) and (struct.fields[i].series != ""): # ...this should be enough, have to give remote series # for ROS msgs, don't have to write local pkg type, but is a good idea
                    fieldseriesName = struct.fields[i].series
                else: # assuming is inside current local set... probably a good assumption
                    #print("otherstruct in same series")
                    #s += rospkgname + "/"
                    fieldseriesName = MDMseriesName
                #print("DEBUG: (rospkgname,struct.name) = (%r,%r)" % (rospkgname, str(struct.name)) )
                s += get_extendsby_strings_for_rosmsg_piece(extendersby,fieldseriesName,str(struct.fields[i].type)) # "struct" slot here only needs ".name" part

                # need to give correct Series before type
                if (struct.fields[i].series != None) and (struct.fields[i].series != ""): # ...this should be enough, have to give remote series # for ROS msgs, don't have to write local pkg type, but is a good idea
                    #print("otherstruct in different _msgs")
                    dep = str(struct.fields[i].series).lower() + "_msgs"
                    s += dep + "/"
                    if not(dep in dependencies):
                        dependencies.append(dep)
                else: # assuming is inside current local set... probably a good assumption
                    #print("otherstruct in same series")
                    #s += rospkgname + "/"
                    s += str(MDMseriesName).lower() + "_msgs/"
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

if __name__ == "__main__":
    main()

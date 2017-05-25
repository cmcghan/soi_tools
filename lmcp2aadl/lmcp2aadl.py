#!/usr/bin/python
#import xml.etree.ElementTree as ET
import lxml.etree as ET
import argparse as AP
import sys
import textwrap


def strip_whitespace(comment):
    return ' '.join(comment.split())

#datatype class
class Data_Type:
    def __init__(self, name, extends, fields):
        self.name = name
        self.fields = fields
        self.extends = extends
        self.comment = None

    def __str__(self):
        s = self.name + ': \n'
        for field in self.fields:
            s += '    ' + str(field) + ' \n'
        return s

    def set_comment(self, text):
        self.comment = strip_whitespace(text)

    __repr__ = __str__

class Field:
    def __init__(self, name, type):
        self.name = name
        self.type = type
        self.is_array = False
        self.comment = None
        self._translate_array_fields()

    def _translate_array_fields(field):
        if field.type.endswith('[]'):
            field.type = field.type[:-2]
            field.is_array = True;

    def __str__(self):
        return self.name + ' : ' + self.type

    def set_comment(self, text):
        self.comment = strip_whitespace(text)

    __repr__ = __str__

class Enum:
    def __init__(self, name, vals):
        self.name = name
        self.vals = vals
        self.comment = None

    def __str__(self):
        return self.name + ' : ' + str(self.vals)

    def set_comment(self, text):
        self.comment = strip_whitespace(text)

    __repr__ = __str__

class XMLCollector(object):
    def __init__(self):
        self.structs = []
        self.enums = []
        self.cur_el = None
        self.cur_fields = None
        self.cur_entries = None
        self.cur_comment = None

    def start(self, tag, attrib):
        if tag == "Struct" :
            name = attrib['Name']
            extends = None
            if 'Extends' in attrib:
                extends = attrib['Extends']
            self.cur_fields = []
            self.cur_el = Data_Type(name, extends, self.cur_fields) 
            self.structs.append(self.cur_el)
        if tag == "Field" :
            name = attrib['Name']
            field_type = attrib['Type']
            self.cur_el = Field(name, field_type)
            self.cur_fields.append(self.cur_el)
        if tag == "Enum" :
            name = attrib['Name']
            self.cur_entries =[]
            self.cur_el = Enum(name, self.cur_entries)
            self.enums.append(self.cur_el)
        if tag == "Entry":
            name = attrib['Name']
            self.cur_entries.append(name)
        if self.cur_comment != None and self.cur_el != None:
            self.cur_el.set_comment(self.cur_comment)

    def end(self, tag):
        self.cur_comment = None
            
    def comment(self, text):
        self.cur_comment = text

    def close(self):
        return

def main():

    #parse all of the arguments
    ap = AP.ArgumentParser(description='Converts an LMCP xml file into AADL datatypes')
    ap.add_argument('input_file', metavar='input', type=str, help='the input LMCP xml file')
    ap.add_argument('output_file', metavar='output', type=str, help='the output AADL file')


    args = ap.parse_args()
    args = vars(args)

    in_file = args.get('input_file')
    out_file = args.get('output_file')

    if in_file == out_file:
        print('the input file and output file must be different names')
        sys.exit(1)

    out = open(out_file, 'w')

    #begin parsing xml
    contents = open(in_file, 'r').read()
    collector = XMLCollector()
    parser = ET.XMLParser(target = collector)
    result = ET.XML(contents, parser)

    for s in collector.structs:
        out.write(struct_to_aadl(s))
    
    for e in collector.enums:
        out.write(enum_to_aadl(e))

    out.close()


def enum_to_aadl(enum):
    ret = '\n'   
    sep = ' '*4
    s = 'data ' + enum.name + ' extends Base_Types::Integer' + ret
    s += sep + 'properties' + ret
    s += sep*2 + 'Enumerators => (\n'
    for key in enum.vals:
        if enum.comment != None:
            s+= to_aadl_comment(enum.comment, sep*3)
        s += sep*3 + '"' + key + '",\n' 
    s = s[:-2] #remove the last comma
    s += ');' + ret
    s += 'end ' + enum.name + ';' + ret*2

    s += 'data implementation ' + enum.name + '.i' + ret
    s += 'end ' + enum.name + '.i;' + ret*2
    return s

def struct_to_aadl(struct):
    ret = '\n'   
    sep = ' '*4
    s = 'data ' + struct.name
    if struct.extends != None:
        s+= ' extends ' + struct.extends
    s+= ret
    s += 'end ' + struct.name + ';' + ret*2

    if struct.comment != None:
        s+= to_aadl_comment(struct.comment, '')
    s += 'data implementation ' + struct.name + '.i'
    if struct.extends != None:
        s+= ' extends ' + struct.extends + '.i'
    s += ret
    s += sep + 'subcomponents' + ret
    for field in struct.fields:
        if field.comment != None:
            s += to_aadl_comment(field.comment, sep*2)
        s += sep*2 + field.name + ': data ' + field.type + '.i'
        if field.is_array:
            s += ' {Data_Representation => Array;}'
        s += ';' + ret
    s += 'end ' + struct.name + '.i;' + ret*2
    return s

def to_aadl_comment(comment, indent):
    return str(indent + ('\n' + indent + '--').join(textwrap.wrap('--' + comment))) + '\n'

main()

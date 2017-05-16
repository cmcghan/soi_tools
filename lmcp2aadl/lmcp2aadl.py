#!/usr/bin/python
import xml.etree.ElementTree as ET
import argparse as AP
import sys



#datatype class
class Data_Type:
    def __init__(self, name, fields):
        self.name = name
        self.fields = fields

    def __str__(self):
        s = self.name + ': \n'
        for field in self.fields:
            s += '    ' + str(field) + ' \n'
        return s

    __repr__ = __str__

class Field:
    def __init__(self, name, type):
        self.name = name
        self.type = type
        self.is_array = False
        self._translate_array_fields()

    def _translate_array_fields(field):
        if field.type.endswith('[]'):
            field.type = field.type[:-2]
            field.is_array = True;

    def __str__(self):
        return self.name + ' : ' + self.type

    __repr__ = __str__

class Enum:
    def __init__(self, name, vals):
        self.name = name
        self.vals = vals

    def __str__(self):
        return self.name + ' : ' + str(self.vals)

    __repr__ = __str__

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
    tree = ET.parse(in_file)
    root = tree.getroot()


    #parse structs
    data_types = []
    for struct_list in root.findall('StructList'):
        for struct in struct_list.findall('Struct'):
            parse_struct(struct, data_types)

    #parse enums
    enums = []
    for enum_list in root.findall('EnumList'):
        for enum in enum_list.findall('Enum'):
            parse_enum(enum, enums)

    for s in data_types:
        out.write(struct_to_aadl(s))
    
    for e in enums:
        out.write(enum_to_aadl(e))

    out.close()



def parse_struct(s, structs):
    fields = []
    for field in s.findall('Field'):
        fields.append(Field(field.get('Name'), field.get('Type')))

    structs.append(Data_Type(s.get('Name'), fields))


def parse_enum(e, enums):
    entries = {}
    for entry in e.findall('Entry'):
        entries[entry.get('Name')] = entry.get('Value')

    enums.append(Enum(e.get('Name'), entries))


def enum_to_aadl(enum):
    ret = '\n'   
    sep = ' '*4
    s = 'data ' + enum.name + ' extends Base_Types::Integer' + ret
    s += sep + 'properties' + ret
    s += sep*2 + 'Enumerators => ('
    delim = ""
    for key in enum.vals:
        s += delim + '"' + key + '"'
        delim = ","
    s += ');' + ret
    s += 'end ' + enum.name + ';' + ret*2

    s += 'data implementation ' + enum.name + '.i' + ret
    s += 'end ' + enum.name + '.i;' + ret*2
    return s

def struct_to_aadl(struct):
    ret = '\n'   
    sep = ' '*4
    s = 'data ' + struct.name + ret
    s += 'end ' + struct.name + ';' + ret*2

    s += 'data implementation ' + struct.name + '.i' + ret
    s += sep + 'subcomponents' + ret
    for field in struct.fields:
        s += sep*2 + field.name + ': data ' + field.type + '.i'
        if field.is_array:
            s += ' {Data_Representation => Array;}'
        s += ';' + ret
    s += 'end ' + struct.name + '.i;' + ret*2
    return s

main()

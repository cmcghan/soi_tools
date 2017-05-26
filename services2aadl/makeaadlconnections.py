#!/usr/bin/python
import lxml.etree as ET
import argparse as AP
import sys
import textwrap
import subprocess
import re


def strip_whitespace(comment):
    return ' '.join(comment.split())

class Connection:
    def __init__(self, name, src_name, dest_name, src_field_name, dst_field_name):
        self.name = strip_whitespace(name)
        self.src_name = strip_whitespace(src_name)
        self.dest_name = strip_whitespace(dest_name)
        self.src_field_name = strip_whitespace(src_field_name)
        self.dst_field_name = strip_whitespace(dst_field_name)
    
    def __str__(self):
        return self.name + " : port " + self.src_name + "." + self.src_field_name + " -> " + self.dest_name + "." + self.dst_field_name + ";"

class Component_Type:
    def __init__(self, name):
        self.name = name
        self.inputs = set([])
        self.outputs =set([])
        self.comment = None

    def __str__(self):
        s = self.name + ': \n'
        s += 'inputs\n'
        for i in self.inputs:
            s+= '    ' + str(i) + '\n'
        s += 'outputs\n'
        for o in self.outputs:
            s+= '    ' + str(o) + '\n'
        return s

    def set_comment(self, text):
        self.comment = strip_whitespace(text)

    __repr__ = __str__

class Field:
    def __init__(self, name, type_name, is_input):
        self.name = name
        self.type = type_name
        self.comment = None
        self.is_input = is_input

    def __str__(self):
        return self.name + ' : ' + self.type

    def set_comment(self, text):
        self.comment = strip_whitespace(text)

    __repr__ = __str__

def main():

    #parse all of the arguments
    ap = AP.ArgumentParser(description='takes an aadl file with process type definitions as input and outputs connections and subcomponent declarations')
    ap.add_argument('in', metavar='input', type=str, help='the input AADL file')
    ap.add_argument('-out', metavar='output', type=str, help='the output AADL file')


    args = ap.parse_args()
    args = vars(args)

    in_file = args.get('in')
    out_file = args.get('out')

    if out_file != None and in_file == out_file:
        print('the input file and output file must be different names')
        sys.exit(1)

    if out_file != None:
        out = open(out_file, 'w')
    else:
        out = sys.stdout

    i = open(in_file, 'r')

    cts = parse(i)

    conns = connections(cts)
    for ct in cts:
        out.write(ct.name + " : process " + ct.name + ";\n")
    for c in conns:
        out.write(str(c) + '\n')
    i.close()
    out.close()

def connections(cts):
    conns = []
    field2ct = {}
    inputs2field = {}
    outputs2field = {}
    for ct in cts:
        for i in ct.inputs:
            field2ct[i] = ct
            l = []
            if i.type in inputs2field.keys():
                l = inputs2field[i.type]
            else:
                inputs2field[i.type] = l
            l.append(i)

        for i in ct.outputs:
            field2ct[i] = ct
            l = []
            if i.type in outputs2field.keys():
                l = outputs2field[i.type]
            else:
                outputs2field[i.type] = l
            l.append(i)

    conn_count = 0
    for k in outputs2field:
        outs = outputs2field[k]
        if k not in inputs2field.keys():
            inputs2field[k] = []
        ins = inputs2field[k]
        for o in outs:
            src = field2ct[o]
            for i in ins:
                dst = field2ct[i]
                conns.append(Connection('conn'+str(conn_count), src.name, dst.name, o.name, i.name))
                conn_count += 1
    return conns


def parse(pipe):

    ct = None
    cts =[]
    for line in pipe.readlines():
        line = line.rstrip()
        if line.startswith("process"):
            name = line.split()[1]
            ct = Component_Type(name)
        if "in event" in line:
            ct.inputs.add(parse_feature(line))
        if "out event" in line:
            ct.outputs.add(parse_feature(line))
        if line.startswith("end "):
            cts.append(ct)
    return cts

def parse_feature(line):
    name = line.split(": ")[0]
    is_input = False
    if "in event" in line:
        is_input = True
    p = re.compile("[^ ]*::.*\.i")
    type_name = p.findall(line)[0]
    f = Field(name, type_name, is_input)
    return f



def get_simple_name(n):
    p = re.compile('[^::]*$')
    return p.findall(n)[0]


def ct_to_aadl(ct):
    ret = '\n'   
    sep = ' '*4
    s = 'process ' + ct.name + ret
    s += sep + 'features' + ret
    index = 0
    for i in ct.inputs:
        s += sep*2 + get_simple_name(i) + '_in : in event data port ' + i + '.i;' + ret
        index += 1
    index = 0
    for i in ct.outputs:
        s += sep*2 + get_simple_name(i) + '_out : out event data port ' + i + '.i;' + ret
        index += 1
    s += 'end ' + ct.name + ';' + ret*2
    return s

def to_aadl_comment(comment, indent):
    return str(indent + ('\n' + indent + '--').join(textwrap.wrap('--' + comment))) + '\n'

main()

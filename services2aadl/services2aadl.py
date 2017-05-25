#!/usr/bin/python
import lxml.etree as ET
import argparse as AP
import sys
import textwrap
import subprocess
import re


def strip_whitespace(comment):
    return ' '.join(comment.split())

#datatype class
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

def main():

    #parse all of the arguments
    ap = AP.ArgumentParser(description='Converts an bitcode file for a serivce of the UxAS into an AADL file for the service')
    ap.add_argument('in', metavar='input', type=str, help='the input bitcode file file')
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

    proc = subprocess.Popen(['opt', '-load', 'LLVMUXAS2AADL.so', '-uxas', in_file], stderr=subprocess.PIPE);

    name = in_file.split('.')[0]
    ct = parse_llvm_pass(name, proc.stderr)
    translate_ct(ct)

    print ct_to_aadl(ct)

    out.close()

def translate_ct(ct):
    p = re.compile('[^::]*::[^::]*$')
    new_inputs = set([])
    for i in ct.inputs:
        i = i.replace('::Subscription', '')
        i = i.replace('()', '')
        i = p.findall(i)[0]
        new_inputs.add(i)

    ct.inputs = new_inputs

    new_outputs = set([])
    for i in ct.outputs:
        i = i.replace('()', '')
        i = p.findall(i)[0]
        new_outputs.add(i)

    ct.outputs = new_outputs


def parse_llvm_pass(name, pipe):

    acc = set([])
    component = Component_Type(name)
    for line in pipe.readlines():
        line = line.rstrip()
        if line == 'inputs':
            acc = component.inputs
        elif line == 'outputs':
            acc = component.outputs
        else:
            acc.add(line)

    return component

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

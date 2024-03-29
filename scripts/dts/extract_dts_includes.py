#!/usr/bin/env python3
#
# Copyright (c) 2017, Linaro Limited
# Copyright (c) 2018, Bobby Noelte
#
# SPDX-License-Identifier: Apache-2.0
#

# vim: ai:ts=4:sw=4

import sys
from os import listdir
import os, fnmatch
import re
import yaml
import argparse
from collections.abc import Mapping
from copy import deepcopy

from devicetree import parse_file
from extract.globals import *
import extract.globals

from extract.clocks import clocks
from extract.compatible import compatible
from extract.interrupts import interrupts
from extract.reg import reg
from extract.flash import flash
from extract.pinctrl import pinctrl
from extract.default import default

class Bindings(yaml.Loader):

    ##
    # List of all yaml files available for yaml loaders
    # of this class. Must be preset before the first
    # load operation.
    _files = []

    ##
    # Files that are already included.
    # Must be reset on the load of every new binding
    _included = []

    @classmethod
    def bindings(cls, compatibles, yaml_dirs):
        # find unique set of compatibles across all active nodes
        s = set()
        for k, v in compatibles.items():
            if isinstance(v, list):
                for item in v:
                    s.add(item)
            else:
                s.add(v)

        # scan YAML files and find the ones we are interested in
        cls._files = []
        for yaml_dir in yaml_dirs:
            for root, dirnames, filenames in os.walk(yaml_dir):
                for filename in fnmatch.filter(filenames, '*.yaml'):
                    cls._files.append(os.path.join(root, filename))

        yaml_list = {}
        yaml_list['node'] = {}
        yaml_list['bus'] = {}
        yaml_list['compat'] = []
        file_load_list = set()
        for file in cls._files:
            for line in open(file, 'r', encoding='utf-8'):
                if re.search('^\s+constraint:*', line):
                    c = line.split(':')[1].strip()
                    c = c.strip('"')
                    if c in s:
                        if file not in file_load_list:
                            file_load_list.add(file)
                            with open(file, 'r', encoding='utf-8') as yf:
                                cls._included = []
                                l = yaml_traverse_inherited(file, yaml.load(yf, cls))
                                if c not in yaml_list['compat']:
                                    yaml_list['compat'].append(c)
                                if 'parent' in l:
                                    bus = l['parent']['bus']
                                    if not bus in yaml_list['bus']:
                                        yaml_list['bus'][bus] = {}
                                    yaml_list['bus'][bus][c] = l
                                else:
                                    yaml_list['node'][c] = l
        return (yaml_list['node'], yaml_list['bus'], yaml_list['compat'])

    def __init__(self, stream):
        filepath = os.path.realpath(stream.name)
        if filepath in self._included:
            print("Error:: circular inclusion for file name '{}'".
                  format(stream.name))
            raise yaml.constructor.ConstructorError
        self._included.append(filepath)
        super(Bindings, self).__init__(stream)
        Bindings.add_constructor('!include', Bindings._include)
        Bindings.add_constructor('!import',  Bindings._include)

    def _include(self, node):
        if isinstance(node, yaml.ScalarNode):
            return self._extract_file(self.construct_scalar(node))

        elif isinstance(node, yaml.SequenceNode):
            result = []
            for filename in self.construct_sequence(node):
                result.append(self._extract_file(filename))
            return result

        elif isinstance(node, yaml.MappingNode):
            result = {}
            for k, v in self.construct_mapping(node).iteritems():
                result[k] = self._extract_file(v)
            return result

        else:
            print("Error:: unrecognised node type in !include statement")
            raise yaml.constructor.ConstructorError

    def _extract_file(self, filename):
        filepaths = [filepath for filepath in self._files if filepath.endswith(filename)]
        if len(filepaths) == 0:
            print("Error:: unknown file name '{}' in !include statement".
                  format(filename))
            raise yaml.constructor.ConstructorError
        elif len(filepaths) > 1:
            # multiple candidates for filename
            files = []
            for filepath in filepaths:
                if os.path.basename(filename) == os.path.basename(filepath):
                    files.append(filepath)
            if len(files) > 1:
                print("Error:: multiple candidates for file name '{}' in !include statement".
                      format(filename), filepaths)
                raise yaml.constructor.ConstructorError
            filepaths = files
        with open(filepaths[0], 'r', encoding='utf-8') as f:
            return yaml.load(f, Bindings)

def extract_single(node_address, prop, key, def_label):

    prop_def = {}
    prop_alias = {}

    if isinstance(prop, list):
        for i, p in enumerate(prop):
            k = convert_string_to_label(key)
            label = def_label + '_' + k
            if isinstance(p, str):
                p = "\"" + p + "\""
            add_compat_alias(node_address,
                    k + '_' + str(i),
                    label + '_' + str(i),
                    prop_alias)
            prop_def[label + '_' + str(i)] = p
    else:
        k = convert_string_to_label(key)
        label = def_label + '_' + k

        if prop == 'parent-label':
            prop = find_parent_prop(node_address, 'label')

        if isinstance(prop, str):
            prop = "\"" + prop + "\""
        add_compat_alias(node_address, k, label, prop_alias)
        prop_def[label] = prop

        # generate defs for node aliases
        if node_address in aliases:
            add_prop_aliases(
                node_address,
                lambda alias:
                    convert_string_to_label(alias) + '_' + k,
                label,
                prop_alias)

    insert_defs(node_address, prop_def, prop_alias)

def extract_string_prop(node_address, key, label):

    prop_def = {}

    node = reduced[node_address]
    prop = node['props'][key]

    k = convert_string_to_label(key)
    prop_def[label] = "\"" + prop + "\""

    if node_address in defs:
        defs[node_address].update(prop_def)
    else:
        defs[node_address] = prop_def


def extract_property(node_compat, node_address, prop, prop_val, names):

    node = reduced[node_address]
    yaml_node_compat = get_binding(node_address)
    if 'base_label' in yaml_node_compat:
        def_label = yaml_node_compat.get('base_label')
    else:
        def_label = get_node_label(node_address)

    if 'parent' in yaml_node_compat:
        if 'bus' in yaml_node_compat['parent']:
            # get parent label
            parent_address = get_parent_address(node_address)

            #check parent has matching child bus value
            try:
                parent_yaml = get_binding(parent_address)
                parent_bus = parent_yaml['child']['bus']
            except (KeyError, TypeError) as e:
                raise Exception(str(node_address) + " defines parent " +
                        str(parent_address) + " as bus master but " +
                        str(parent_address) + " not configured as bus master " +
                        "in yaml description")

            if parent_bus != yaml_node_compat['parent']['bus']:
                bus_value = yaml_node_compat['parent']['bus']
                raise Exception(str(node_address) + " defines parent " +
                        str(parent_address) + " as " + bus_value +
                        " bus master but " + str(parent_address) +
                        " configured as " + str(parent_bus) +
                        " bus master")

            # Generate alias definition if parent has any alias
            if parent_address in aliases:
                for i in aliases[parent_address]:
                    # Build an alias name that respects device tree specs
                    node_name = node_compat + '-' + node_address.split('@')[-1]
                    node_strip = node_name.replace('@','-').replace(',','-')
                    node_alias = i + '-' + node_strip
                    if node_alias not in aliases[node_address]:
                        # Need to generate alias name for this node:
                        aliases[node_address].append(node_alias)

            # Use parent label to generate label
            parent_label = get_node_label(parent_address)
            def_label = parent_label + '_' + def_label

            # Generate bus-name define
            extract_single(node_address, 'parent-label',
                           'bus-name', 'DT_' + def_label)

    if 'base_label' not in yaml_node_compat:
        def_label = 'DT_' + def_label

    if prop == 'reg':
        reg.extract(node_address, names, def_label, 1)
    elif prop == 'interrupts' or prop == 'interrupts-extended':
        interrupts.extract(node_address, prop, names, def_label)
    elif prop == 'compatible':
        compatible.extract(node_address, prop, def_label)
    elif 'pinctrl-' in prop:
        pinctrl.extract(node_address, prop, def_label)
    elif 'clocks' in prop:
        clocks.extract(node_address, prop, def_label)
    elif 'pwms' in prop or 'gpios' in prop:
        # drop the 's' from the prop
        generic = prop[:-1]
        try:
            prop_values = list(reduced[node_address]['props'].get(prop))
        except:
            prop_values = reduced[node_address]['props'].get(prop)

        extract_controller(node_address, prop, prop_values, 0,
                           def_label, generic)
        extract_cells(node_address, prop, prop_values,
                      names, 0, def_label, generic)
    else:
        default.extract(node_address, prop, prop_val['type'], def_label)


def extract_node_include_info(reduced, root_node_address, sub_node_address,
                              y_sub):

    filter_list = ['interrupt-names',
                    'reg-names',
                    'phandle',
                    'linux,phandle']
    node = reduced[sub_node_address]
    node_compat = get_compat(root_node_address)

    if node_compat not in get_binding_compats():
        return {}, {}

    if y_sub is None:
        y_node = get_binding(root_node_address)
    else:
        y_node = y_sub

    # check to see if we need to process the properties
    for k, v in y_node['properties'].items():
            if 'properties' in v:
                for c in reduced:
                    if root_node_address + '/' in c:
                        extract_node_include_info(
                            reduced, root_node_address, c, v)
            if 'generation' in v:

                match = False

                # Handle any per node extraction first.  For example we
                # extract a few different defines for a flash partition so its
                # easier to handle the partition node in one step
                if 'partition@' in sub_node_address:
                    flash.extract_partition(sub_node_address)
                    continue

                # Handle each property individually, this ends up handling common
                # patterns for things like reg, interrupts, etc that we don't need
                # any special case handling at a node level
                for c in node['props'].keys():
                    # if prop is in filter list - ignore it
                    if c in filter_list:
                        continue

                    if re.match(k + '$', c):

                        if 'pinctrl-' in c:
                            names = deepcopy(node['props'].get(
                                                        'pinctrl-names', []))
                        else:
                            if not c.endswith("-names"):
                                names = deepcopy(node['props'].get(
                                                        c[:-1] + '-names', []))
                                if not names:
                                    names = deepcopy(node['props'].get(
                                                            c + '-names', []))
                        if not isinstance(names, list):
                            names = [names]

                        extract_property(
                            node_compat, sub_node_address, c, v, names)
                        match = True

                # Handle the case that we have a boolean property, but its not
                # in the dts
                if not match:
                    if v['type'] == "boolean":
                        extract_property(
                            node_compat, sub_node_address, k, v, None)

def dict_merge(parent, fname, dct, merge_dct):
    # from https://gist.github.com/angstwad/bf22d1822c38a92ec0a9

    """ Recursive dict merge. Inspired by :meth:``dict.update()``, instead of
    updating only top-level keys, dict_merge recurses down into dicts nested
    to an arbitrary depth, updating keys. The ``merge_dct`` is merged into
    ``dct``.
    :param parent: parent tuple key
    :param fname: yaml file being processed
    :param dct: dict onto which the merge is executed
    :param merge_dct: dct merged into dct
    :return: None
    """
    for k, v in merge_dct.items():
        if (k in dct and isinstance(dct[k], dict)
                and isinstance(merge_dct[k], Mapping)):
            dict_merge(k, fname, dct[k], merge_dct[k])
        else:
            if k in dct and dct[k] != merge_dct[k]:
                merge_warn = True
                # Don't warn if we are changing the "category" from "optional
                # to "required"
                if (k == "category" and dct[k] == "optional" and
                    merge_dct[k] == "required"):
                    merge_warn = False
                if merge_warn:
                    print("extract_dts_includes.py: {}('{}') merge of property '{}': '{}' overwrites '{}'.".format(
                            fname, parent, k, merge_dct[k], dct[k]))
            dct[k] = merge_dct[k]


def yaml_traverse_inherited(fname, node):
    """ Recursive overload procedure inside ``node``
    ``inherits`` section is searched for and used as node base when found.
    Base values are then overloaded by node values
    and some consistency checks are done.
    :param fname: initial yaml file being processed
    :param node:
    :return: node
    """

    # do some consistency checks. Especially id is needed for further
    # processing. title must be first to check.
    if 'title' not in node:
        # If 'title' is missing, make fault finding more easy.
        # Give a hint what node we are looking at.
        print("extract_dts_includes.py: node without 'title' -", node)
    for prop in ('title', 'version', 'description'):
        if prop not in node:
            node[prop] = "<unknown {}>".format(prop)
            print("extract_dts_includes.py: '{}' property missing".format(prop),
                  "in '{}' binding. Using '{}'.".format(node['title'], node[prop]))

    # warn if we have an 'id' field
    if 'id' in node:
        print("extract_dts_includes.py: WARNING: id field set",
              "in '{}', should be removed.".format(node['title']))

    if 'inherits' in node:
        if isinstance(node['inherits'], list):
            inherits_list  = node['inherits']
        else:
            inherits_list  = [node['inherits'],]
        node.pop('inherits')
        for inherits in inherits_list:
            if 'inherits' in inherits:
                inherits = yaml_traverse_inherited(fname, inherits)
            # title, description, version of inherited node
            # are overwritten by intention. Remove to prevent dct_merge to
            # complain about duplicates.
            inherits.pop('title', None)
            inherits.pop('version', None)
            inherits.pop('description', None)
            dict_merge(None, fname, inherits, node)
            node = inherits
    return node


def get_key_value(k, v, tabstop):
    label = "#define " + k

    # calculate the name's tabs
    if len(label) % 8:
        tabs = (len(label) + 7) >> 3
    else:
        tabs = (len(label) >> 3) + 1

    line = label
    for i in range(0, tabstop - tabs + 1):
        line += '\t'
    line += str(v)
    line += '\n'

    return line


def output_keyvalue_lines(fd):
    node_keys = sorted(defs.keys())
    for node in node_keys:
        fd.write('# ' + node.split('/')[-1])
        fd.write("\n")

        prop_keys = sorted(defs[node].keys())
        for prop in prop_keys:
            if prop == 'aliases':
                for entry in sorted(defs[node][prop]):
                    a = defs[node][prop].get(entry)
                    fd.write("%s=%s\n" % (entry, defs[node].get(a)))
            else:
                fd.write("%s=%s\n" % (prop, defs[node].get(prop)))

        fd.write("\n")

def generate_keyvalue_file(kv_file):
    with open(kv_file, "w") as fd:
        output_keyvalue_lines(fd)


def output_include_lines(fd):
    fd.write('''\
/**********************************************
*                 Generated include file
*                      DO NOT MODIFY
*/
#ifndef GENERATED_DTS_BOARD_UNFIXED_H
#define GENERATED_DTS_BOARD_UNFIXED_H
    '''
    )

    node_keys = sorted(defs.keys())
    for node in node_keys:
        fd.write('/* ' + node.split('/')[-1] + ' */')
        fd.write("\n")

        max_dict_key = lambda d: max(len(k) for k in d.keys())
        maxlength = 0
        if defs[node].get('aliases'):
            maxlength = max_dict_key(defs[node]['aliases'])
        maxlength = max(maxlength, max_dict_key(defs[node])) + len('#define ')

        if maxlength % 8:
            maxtabstop = (maxlength + 7) >> 3
        else:
            maxtabstop = (maxlength >> 3) + 1

        if (maxtabstop * 8 - maxlength) <= 2:
            maxtabstop += 1

        prop_keys = sorted(defs[node].keys())
        for prop in prop_keys:
            if prop == 'aliases':
                for entry in sorted(defs[node][prop]):
                    a = defs[node][prop].get(entry)
                    fd.write(get_key_value(entry, a, maxtabstop))
            else:
                fd.write(get_key_value(prop, defs[node].get(prop), maxtabstop))

        fd.write("\n")

    fd.write("#endif\n")


def generate_include_file(inc_file):
    with open(inc_file, "w") as fd:
        output_include_lines(fd)


def load_and_parse_dts(dts_file):
    with open(dts_file, "r", encoding="utf-8") as fd:
        dts = parse_file(fd)

    return dts


def load_yaml_descriptions(dts, yaml_dirs):
    compatibles = get_all_compatibles(dts['/'], '/', {})

    (bindings, bus, bindings_compat) = Bindings.bindings(compatibles, yaml_dirs)
    if not bindings:
        raise Exception("Missing YAML information.  Check YAML sources")

    return (bindings, bus, bindings_compat)


def generate_node_definitions():

    for k, v in reduced.items():
        node_compat = get_compat(k)
        if node_compat is not None and node_compat in get_binding_compats():
            extract_node_include_info(reduced, k, k, None)

    if defs == {}:
        raise Exception("No information parsed from dts file.")

    for k, v in regs_config.items():
        if k in chosen:
            reg.extract(chosen[k], None, v, 1024)

    for k, v in name_config.items():
        if k in chosen:
            extract_string_prop(chosen[k], "label", v)

    node_address = chosen.get('zephyr,flash', 'dummy-flash')
    flash.extract(node_address, 'zephyr,flash', 'FLASH')
    node_address = chosen.get('zephyr,code-partition', node_address)
    flash.extract(node_address, 'zephyr,code-partition', 'FLASH')

    return defs


def parse_arguments():
    rdh = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=rdh)

    parser.add_argument("-d", "--dts", nargs=1, required=True, help="DTS file")
    parser.add_argument("-y", "--yaml", nargs='+', required=True,
                        help="YAML file directories, we allow multiple")
    parser.add_argument("-i", "--include", nargs=1,
                        help="Generate include file for the build system")
    parser.add_argument("-k", "--keyvalue", nargs=1,
                        help="Generate config file for the build system")
    parser.add_argument("--old-alias-names", action='store_true',
                        help="Generate aliases also in the old way, without "
                             "compatibility information in their labels")
    return parser.parse_args()


def main():
    args = parse_arguments()
    enable_old_alias_names(args.old_alias_names)

    dts = load_and_parse_dts(args.dts[0])

    # build up useful lists
    get_reduced(dts['/'], '/')
    get_phandles(dts['/'], '/', {})
    get_aliases(dts['/'])
    get_chosen(dts['/'])

    (extract.globals.bindings, extract.globals.bus_bindings,
     extract.globals.bindings_compat) = load_yaml_descriptions(dts, args.yaml)

    defs = generate_node_definitions()

    # Add DT_CHOSEN_<X> defines to generated files
    for c in sorted(chosen.keys()):
        chosen_def = 'DT_CHOSEN_' + convert_string_to_label(c)
        load_defs = {
            chosen_def: "1",
        }
        insert_defs('chosen', load_defs, {})

     # generate config and include file
    if (args.keyvalue is not None):
       generate_keyvalue_file(args.keyvalue[0])

    if (args.include is not None):
       generate_include_file(args.include[0])


if __name__ == '__main__':
    main()

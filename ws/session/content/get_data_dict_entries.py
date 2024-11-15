#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2024-04-03
#
#
# ---------------------------------------------------------------------

import argparse
import json
import re

def remove_comments(json_like):
    """Remove C-style comments from a JSON-like string."""
    pattern = r'//.*?$|/\*.*?\*/'
    return re.sub(pattern, '', json_like, flags=re.DOTALL | re.MULTILINE)

def json_to_dict(path_to_json):
    with open(path_to_json, 'r') as file:
        content = file.read()

        content_no_comments = remove_comments(content)
        # Return parsed JSON
        return json.loads(content_no_comments)

def get_data_dict_entry(key):
    data_dict = json_to_dict('/data_dict.json')
    if isinstance(key, list):
        # Nested parameter case
        result = data_dict
        for key in key:
            if key in result:
                result = result[key]
            else:
                # Key not found in data dict, will use the literal string instead
                result = key
    elif isinstance(key, str):
        # Top-level parameter case
        if key in data_dict:
            result = data_dict[key]
        else:
            # Key not found in data dict, will use the literal string instead
            result = key
    else:
        raise TypeError("target_account_key must be either a string or a list of strings.")
    return result

def main(key_string):
    # Check if the string contains a semicolon
    if '+' in key_string:
        # Split the string by semicolon and then split each part by spaces
        keys = [part.strip().split() for part in key_string.split('+')]
    else:
        # Split the string by spaces
        keys = [key_string.split()]
    resolved_keys = [get_data_dict_entry(key) for key in keys]
    return resolved_keys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Script that resolves keys of the ips_and_accounts.json')
    parser.add_argument('-k', '--key_string', required=True, help='String that contains keys')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    resolved_data_str = ",".join(main(**{k: v for k, v in vars(args).items() if v is not None}))

    print(resolved_data_str)  # This will output the value to stdout, which can be captured in a shell

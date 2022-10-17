import re
import struct
try:
    from demangler import demangle
except Exception as e:
    demangle = None


def hex_to_float(h):
    return struct.unpack('!f', bytes.fromhex(h))[0]

def remove_tabs(txt):
    for i in range(len(txt)):
        if txt[i] not in ["\t", " "]:
            return txt[i:], i


def readlines(path):
    with open(path, "r") as f:
        return f.readlines()    

def annotate_ptx(path):
    lines = readlines(path)

    # extract file annotations
    files = dict()
    for line in lines:
        line = line.strip(" \t\n")
        
        m = re.match(r'\.file\s*([0-9]+)\s*"([^"]+)".*', line)
        if m:
            files[m.group(1)] = readlines(m.group(2))

    kernels = []
    dst_lines = []

    for line in lines:
        line = line.strip(" \t\n")

        if demangle is not None:
            # add comments with demangled kernel names
            m = re.match(r'\.visible\s*\.entry\s*([^(]+)', line)
            if m:
                kernel = demangle(m.group(1))
                line = f"// {kernel}\n{line}"
                kernels.append(kernel)
        
        # make .loc annotations explicit adding the corresponding source file line 
        m = re.match(r'\.loc\s*([0-9]+)\s*([0-9]+)\s*([0-9]+)', line)
        if m:
            file_id = m.group(1)
            line_n = int(m.group(2))
            column = int(m.group(3))
            src_txt, n_tabs = remove_tabs(files[file_id][line_n-1])
            column -= n_tabs + 1
            line = f"\n// {src_txt}// {' '*column}^\n{line}"
            
        # annotate hex float writing them in decimal format
        m = re.match(r'.*0f([0-9a-fA-F]{8})', line)
        if m:
            hexv = m.group(1)
            line = f"// {hexv} = {hex_to_float(hexv)}f\n{line}"
        
        dst_lines.append(line)

    with open(path, "w") as f:
        f.write("\n".join(dst_lines))
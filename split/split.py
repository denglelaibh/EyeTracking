import re
import csv

def find_num(text):
    return list(map(
        lambda x: float(x),
        text.split(",")
    ))

f=open("/Users/denglelai/Documents/fixHeadLog.txt")
chunks = []
chunk = []
for line in f:
    if line.find("glints") is not -1 and len(chunk) is not 0:
        chunks.append('\t'.join(chunk))
        chunk = []
    else:
        chunk.append(line)

outfile = csv.writer(open("data.csv", "wb"))
outfile.writerow(['leftgx','leftgy','rightgx','rightgy','left0x','left0y','leftix','leftiy','right0x','right0y','rightix','rightiy'])

for chunk_str in chunks:
    row_list = []
    for num_text in re.findall(r"[0-9.]+, [0-9.]+", chunk_str):
        for num in find_num(num_text):
            row_list.append(num)
    outfile.writerow(row_list)
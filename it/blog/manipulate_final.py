#!/usr/bin/env python
# license removed for brevity
import os
newfilenames = list()
list = os.listdir("./_posts")

for file in list:   
    name = file.split(".")
    name = name[0]
    name = name.split("-")
    filedate = name[0:3]
    filename = name[3:]
    newfilenames = "/"+"/".join(filedate)+"/"+"-".join(filename)+"/"
    os.chdir("/home/fiorella/devel/hbr-blog/hotblackrobotics.github.io/it/blog/_posts")
    redirect = "redirect_from:"
    title = "title:"
    redindex = 0

    with open(file, "r") as f:
        lines = f.readlines()

    try:
        # take first element of list, (just one, it just unpackets it)
        redindex = [i for i, line in enumerate(lines) if redirect in line][0]
    except:
        redindex = None

    if redindex == None:
        for num, line in enumerate(lines):
                if title in line:
                    lines.insert(num+1,"redirect_from:\n - "+str(newfilenames)+"\n")
    else:
        if len([1 for line in lines if newfilenames in line]) == 0:
            lines.insert(redindex+1," - "+str(newfilenames)+"\n")

    with open(file, 'w') as f:
        f.writelines(lines)




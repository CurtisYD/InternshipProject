import glob
import os
from array import*
import cv2


os.chdir(r"/home/cydd/OIDv4_ToolKit/OID/Dataset/validation/Door")

filenames = [i for i in glob.glob(f"*.txt")]
myim = [i for i in glob.glob(f"*.jpg")] 
filenames.sort()
myim.sort()



for i in range(0,len(filenames)):

	test = filenames[i]
	f= open(test,"rt")
	data = f.read()
	data = data.replace('Door', '1')
	f.close()
	f = open(test,"wt") 
	f.write(data) 
	f.close() 

	l1 = []
	with open(test, 'r') as fp:
		l1 = fp.readlines()
	with open(test, 'w') as fp:
		for number, line in enumerate(l1):
			if number not in [1,2,3,4]:
				fp.write(line)
	
	images=myim[i]
	im = cv2.imread(myim[i]) 
	h, w, _ = im.shape 
	size = []
	size.append(w)
	size.append(h)

	coor = []
	
	with open(test,'r') as recup:

		# reading each line	
		for line in recup:

			# reading each word		
			for word in line.split():
			
				coor.append(word)
				# displaying the words
						
	print(images)			
	print(test)
	sa = coor[1]
	a = float(sa)
	sb = coor[3]
	b = float(sb)
	sc = coor[2]
	c = float(sc)
	sd = coor[4]
	d = float(sd)
	
	

	box = []
	box.append(a)
	box.append(b)
	box.append(c)
	box.append(d)
	print(box)
	print(size)


	dw = 1./size[0]
	dh = 1./size[1]
	fx = (box[0] + box[1])/2.0
	fy = (box[2] + box[3])/2.0
	fw = box[1] - box[0]
	fh = box[3] - box[2]

	fx = fx*dw
	x = str(fx)
	fw =fw*dw
	w = str(fw)
	fy = fy*dh
	y = str(fy)
	fh = fh*dh
	h = str(fh)
	  

	#replace text in a file
	s = ("1 " +" "+ x +" "+ y +" "+ w +" "+ h)
	f = open(test, "r+")


	f.truncate(0)
	f.write(s)
	f.close()
	print("Text successfully replaced")

	


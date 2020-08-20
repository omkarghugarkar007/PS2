import numpy as np

good = 12 # cell in which COM of bot will be

A = np.zeros((25,24)) # Current Array
B = np.zeros((25,24)) # Next Array

def checking(good,current_row):

	#check: [good,current_row+1],[good+1,current_row+1], [good+1,current_row], [good-1,current_row], [good-1,current_row+1], [good-1,current_row-1], [good,current_row-1], [good+1,current_row-1]
	#good_check_side(good,current_row+1)
	if B[good,current_row+5]==0:
		checkmark = checkmark + 1
	#good_check_side(good+1,current_row)
	if B[good+5,current_row+1]==0:
		checkmark = checkmark + 1
	#good_check_side(good-1,current_row)
	if B[good-5,current_row+1]==0:
		checkmark = checkmark + 1
	#good_check_side(good,current_row-1)
	if B[good,current_row-5]==0:
		checkmark = checkmark + 1
	#good_check_diagonal(good+1,current_row+1)
	if B[good+4,current_row+4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good-1,current_row+1)
	if B[good-4,current_row+4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good+1,current_row-1)
	if B[good+4,current_row-4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good-1,current_row-1)
	if B[good-4,current_row-4] ==0:
		checkmark = checkmark+1


def Create_B_row(good_prev,current_row):

	#Working on B[good,current_row-4] i.e checking the next cell of the column
	checkmark = 0
	checking(good_prev,current_row+1)
	if checkmark == 8:
		B[good_prev,current_row+1]=1
		return

	checkmark=0
	checking(good_prev+1,current_row+1)
	if checkmark==8:
		B[good_prev+1,current_row+1]=1
		good_prev=good_prev+1
		return

	checkmark=0
	checking(good_prev-1,current_row+1)
	if checkmark==8:
		B[good_prev-1,current_row+1]
		good_prev=good_prev-1
		return

	print("No Path in Front")

	#Marking row as checked
	B[24,current_row+1]=1

def move():

	#Initial Moment or Turning time
	if np.sum(A) = 0:

		for i in range(25):

			try:
				Create_B_row(good_prev,i)#turn

			except:
				Create_B_row(good,i)#initial
				

	#Consider When Bot is in Row x of A where x belongs to 0 to 4
	else:

		



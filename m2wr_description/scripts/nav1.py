import numpy as np

good = 12 # cell in which COM of bot will be
good_prev = 12

B = np.zeros((24,24))
New_B =np.zeros((24,24))

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


def Create_row(good_prev,current_row):

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


def checking_New(good,current_row):

	#check: [good,current_row+1],[good+1,current_row+1], [good+1,current_row], [good-1,current_row], [good-1,current_row+1], [good-1,current_row-1], [good,current_row-1], [good+1,current_row-1]
	#good_check_side(good,current_row+1)
	if New_B[good,current_row+5]==0:
		checkmark = checkmark + 1
	#good_check_side(good+1,current_row)
	if New_B[good+5,current_row+1]==0:
		checkmark = checkmark + 1
	#good_check_side(good-1,current_row)
	if New_B[good-5,current_row+1]==0:
		checkmark = checkmark + 1
	#good_check_side(good,current_row-1)
	if New_B[good,current_row-5]==0:
		checkmark = checkmark + 1
	#good_check_diagonal(good+1,current_row+1)
	if New_B[good+4,current_row+4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good-1,current_row+1)
	if New_B[good-4,current_row+4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good+1,current_row-1)
	if New_B[good+4,current_row-4] ==0:
		checkmark = checkmark+1
	#good_check_diagonal(good-1,current_row-1)
	if New_B[good-4,current_row-4] ==0:
		checkmark = checkmark+1


def Create_row_New(good_prev,current_row):

	#Working on B[good,current_row-4] i.e checking the next cell of the column
	checkmark = 0
	checking(good_prev,current_row+1)
	if checkmark == 8:
		New_B[good_prev,current_row+1]=1
		B = New_B
		return

	checkmark=0
	checking(good_prev+1,current_row+1)
	if checkmark==8:
		New_B[good_prev+1,current_row+1]=1
		good_prev=good_prev+1
		B = New_B
		return

	checkmark=0
	checking(good_prev-1,current_row+1)
	if checkmark==8:
		New_B[good_prev-1,current_row+1]
		good_prev=good_prev-1
		B = New_B
		return

	print("No Path in Front")


def move():

	if np.sum(A) == 0:

		B[good_prev,0] = 1
		B[good_prev,1] = 1
		B[good_prev,2] = 1
		B[good_prev,3] = 1


		for i in range (5,25):

			Create_row(good_prev,i-5)

	else:

		New_B[:23,:23] = B[1:,1:]
		#will Come from CV
		New_B[:,24] = 

		Create_row_New(20)




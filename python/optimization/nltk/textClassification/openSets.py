import io,csv
import sys
reload(sys)
sys.setdefaultencoding('utf8')

test = csv.reader(io.open("test.csv","r",encoding='utf-8'),skipinitialspace=True)


# Training Set
pos_training = io.open("/var/lib/mysql-files/TRAIN_POS_G_trust2012_4@hotmail.it.csv","r",encoding='utf-8')
neg_training = io.open("/var/lib/mysql-files/TRAIN_POS_B_trust2012_4@hotmail.it.csv","r",encoding='utf-8')

# Test Set
pos_test = io.open("/var/lib/mysql-files/TEST_POS_trust2012@hotmail.it_all.csv","r",encoding='utf-8')
neg_test = io.open("/var/lib/mysql-files/TEST_NEG_trust2012@hotmail.it_all.csv","r",encoding='utf-8')


## Append tag to sets
posTr = csv.reader(pos_training,skipinitialspace=True)
data = list(posTr)
test = list(test)
for w in test:
	print "New Row: "
	print w[0]
	#for h in w:
	       #print h
		
	#print "obj num: ", len(w), "\tnew line"
print len(test)

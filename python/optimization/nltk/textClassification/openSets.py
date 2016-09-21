import io,csv
import sys
reload(sys)
sys.setdefaultencoding('utf8')

test = csv.reader(io.open("test.csv","r",encoding='utf-8'),skipinitialspace=True)


# Training Set
pos_training = io.open("/var/lib/mysql-files/TRAIN_POS_G_trust2012_4@hotmail.it.csv","r",encoding='utf-8').read()
neg_training = io.open("/var/lib/mysql-files/TRAIN_POS_B_trust2012_4@hotmail.it.csv","r",encoding='utf-8').read()


# Test Set
pos_test = io.open("/var/lib/mysql-files/TEST_POS_trust2012@hotmail.it_all.csv","r",encoding='utf-8').read()
neg_test = io.open("/var/lib/mysql-files/TEST_NEG_trust2012@hotmail.it_all.csv","r",encoding='utf-8').read()


## Append tag to sets
posTr = csv.reader(pos_training,delimiter=",",lineterminator="\n",dialect='excel',skipinitialspace=True)
data = list(posTr)
test = list(test)
try:
	for w in test:
		content = w
	        print content
		#print "obj num: ", len(w), "\tnew line"
except csv.Error, e:
	sys.exit('file %s, line %d: %s' % (filename, reader.line_num, e))
print len(test)

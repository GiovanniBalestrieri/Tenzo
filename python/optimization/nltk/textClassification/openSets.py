import io,csv,nltk
import sys
reload(sys)
sys.setdefaultencoding('utf8')
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords


test = csv.reader(io.open("test.csv","r",encoding='utf-8'),skipinitialspace=True)


# Training Set
pos_training = io.open("/var/lib/mysql-files/TRAIN_POS_G_trust2012_4@hotmail.it.csv","r",encoding='utf-8')
neg_training = io.open("/var/lib/mysql-files/TRAIN_POS_B_trust2012_4@hotmail.it.csv","r",encoding='utf-8')

# Test Set
pos_test = io.open("/var/lib/mysql-files/TEST_POS_trust2012@hotmail.it_all.csv","r",encoding='utf-8')
neg_test = io.open("/var/lib/mysql-files/TEST_NEG_trust2012@hotmail.it_all.csv","r",encoding='utf-8')

#f = open('temp','w')
#f.write('hi there\n')
#f.close() 

posTr = csv.reader(pos_training,skipinitialspace=True)
negTr = csv.reader(neg_training,skipinitialspace=True)
posTt = csv.reader(pos_test,skipinitialspace=True)
negTt = csv.reader(neg_test,skipinitialspace=True)

posTrData = list(posTr)
posTtData = list(posTt)
negTrData = list(negTr)
negTtData = list(negTt)
#test = list(test)

augmentedTitlesPos = []
augmentedTitlesNeg = []

for w in posTrData:
	#print "New Row: "
	w[0] = w[0] + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesPos.append(w[0].lower())

for w in negTrData:
	#print "New Row: "
	w[0] = w[0] + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesNeg.append(w[0].lower())

print "Number of Training titles (+/-): ",len(augmentedTitlesPos)," ; ",len(augmentedTitlesNeg)

#print "all words: "
#for a in augmentedTitlesPos:
#	print a 

## Append tag to sets
# This doc will store the training set
doc = []
for r in augmentedTitlesPos:
	doc.append((r,"pos"))

for r in augmentedTitlesNeg:
	doc.append((r,"neg"))

# Check whether doc contains all labelled instances	
#for a in doc:
#	print a

## Prepare wordset
all_words = []

for i in augmentedTitlesPos:
	for a in word_tokenize(i):
		all_words.append(a)

for i in augmentedTitlesNeg:
	for a in word_tokenize(i):
		all_words.append(a)
	
print "Number of words: ",len(all_words)

sit = set(stopwords.words('italian'))
sit.add("+")
sit.add("-")
sit.add("&")
sit.add("e")
sit.add("/")
sit.add("#")

print "removing stopwords IT ..."
all_clean = []
for w in all_words:
	if w not in sit:
		all_clean.append(w)

all_words = all_clean
sen = set(stopwords.words('english'))
print "removing stopwords EN ..."
all_clean = []
for w in all_words:
	if w not in sen:
		all_clean.append(w)

all_words = all_clean

print "Length: ",len(all_words)

# convert to nltk frequency distribution
all_words = nltk.FreqDist(all_words)
print("\t\t\tReally??")
# Show most common words
print(all_words.most_common(15))
# See how many times the word really is present in the reviews
print("Presence of the word 'really':", all_words["colosseo"])


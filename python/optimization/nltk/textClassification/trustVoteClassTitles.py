import io,csv,nltk,sys,random
reload(sys)
sys.setdefaultencoding('utf8')
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords

from nltk.classify.scikitlearn import SklearnClassifier
from sklearn.naive_bayes import MultinomialNB, GaussianNB ,BernoulliNB
from sklearn.linear_model import LogisticRegression, SGDClassifier
from sklearn.svm import SVC,LinearSVC,NuSVC


# Vars:

linSvm = True
naiveB = True
gauSvm = True
mnb = True
lrc = True
sgd = True
nuSvc = False

# Optional paramenters, recurrence of words in Training set in critical
removeDuplicateTrain = False
removeDuplicateTest = True

# Training Set
pos_training = io.open("/var/lib/mysql-files/TRAIN_POS_G_trust2012_4@hotmail.it_yes.csv","r",encoding='utf-8')
neg_training = io.open("/var/lib/mysql-files/TRAIN_NEG_trust2012_4@hotmail.it_yes.csv","r",encoding='utf-8')

# Test Set
pos_test = io.open("/var/lib/mysql-files/TEST_POS_trust2012@hotmail.it_yes.csv","r",encoding='utf-8')
neg_test = io.open("/var/lib/mysql-files/TEST_NEG_trust2012@hotmail.it_yes.csv","r",encoding='utf-8')

posTr = csv.reader(pos_training,skipinitialspace=True)
negTr = csv.reader(neg_training,skipinitialspace=True)
posTt = csv.reader(pos_test,skipinitialspace=True)
negTt = csv.reader(neg_test,skipinitialspace=True)

posTrData = list(posTr)
posTtData = list(posTt)
negTrData = list(negTr)
negTtData = list(negTt)

augmentedTitlesPos = []
augmentedTitlesNeg = []

augmentedTitlesPosTest = []
augmentedTitlesNegTest = []

for w in posTrData:
	#print "New Row: "
	w[0] = w[0]# + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesPos.append(w[0].lower())

for w in negTrData:
	#print "New Row: "
	w[0] = w[0]# + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesNeg.append(w[0].lower())

print "Number of Training titles (+/-): ",len(augmentedTitlesPos)," ; ",len(augmentedTitlesNeg)

#print "all words: "
#for a in augmentedTitlesPos:
#	print a 

if removeDuplicateTrain:
	posTrTmp = set(augmentedTitlesPos)
	negTrTmp = set(augmentedTitlesNeg)
	

	seenTrPos = set()
	result = []
	for item in posTrTmp:
	    if item not in seenTrPos:
	        seenTrPos.add(item)
	        result.append(item)
	augmentedTitlesPos = result

	seenTrNeg = set()
	result = []
	for item in negTrTmp:
	    if item not in seenTrNeg:
	        seenTrNeg.add(item)
        	result.append(item)
	augmentedTitlesNeg = result

print " Length after: ", len(augmentedTitlesPos), " , ", len(augmentedTitlesNeg)


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
sit.add("]")

sit.add("''")
sit.add(";")
sit.add(",")
sit.add(".")
sit.add(":")
sit.add("[")
sit.add("|")
sit.add("!")
sit.add(")")

sit.add("(")
print "removing stopwords IT ..."
all_clean = []
for w in all_words:
	if w not in sit:
		all_clean.append(w)

all_words = all_clean
all_clean = []
sen = set(stopwords.words('english'))
print "removing stopwords EN ..."
for w in all_words:
	if w not in sen:
		all_clean.append(w)

all_words = all_clean

print "Length: ",len(all_words)

#Preparing the Test Set
print "\t\t\t\tTest Set initialization"
for w in posTtData:
	#print "New Row: "
	w[0] = w[0] + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesPosTest.append(w[0].lower())

for w in negTtData:
	#print "New Row: "
	w[0] = w[0] + " " +w[1] +" " +  w[2]
	#print w[0]
	augmentedTitlesNegTest.append(w[0].lower())

print "Number of Test titles (+/-): ",len(augmentedTitlesPosTest)," ; ",len(augmentedTitlesNegTest)

if removeDuplicateTest:
	posTtTmp = set(augmentedTitlesPosTest)
	negTtTmp = set(augmentedTitlesNegTest)

	seenTtPos = set()
	result = []
	for item in posTtTmp:
	    if item not in seenTtPos:
	        seenTtPos.add(item)
	        result.append(item)
	augmentedTitlesPosTest = result

	seenTtNeg = set()
	result = []
	for item in negTtTmp:
	    if item not in seenTtNeg:
        	seenTtNeg.add(item)
	        result.append(item)
	augmentedTitlesNegTest = result

print "After ->Number of Test titles (+/-): ",len(augmentedTitlesPosTest)," ; ",len(augmentedTitlesNegTest)


## Append tag to sets
# This doc will store the training set
docTest = []
for r in augmentedTitlesPosTest:
	docTest.append((r,"pos"))

for r in augmentedTitlesNegTest:
	docTest.append((r,"neg"))

# Check whether doc contains all labelled instances	
#for a in doc:
#	print a

## Prepare wordset
all_words_testSet = []

for i in augmentedTitlesPosTest:
	for a in word_tokenize(i):
		all_words_testSet.append(a)

for i in augmentedTitlesNegTest:
	for a in word_tokenize(i):
		all_words_testSet.append(a)
	
print "Number of words in TestSet: ",len(all_words_testSet)

print "removing stopwords IT ..."
all_clean_testSet = []
for w in all_words_testSet:
	if w not in sit:
		all_clean_testSet.append(w)

all_words_testSet = all_clean_testSet

all_clean_testSet = []

print "removing stopwords EN ..."
for w in all_words_testSet:
	if w not in sen:
		all_clean_testSet.append(w)

all_words_testSet = all_clean_testSet
print "Length: ",len(all_words_testSet)

# convert to nltk frequency distribution
all_words = nltk.FreqDist(all_words)
print("\t\t\tMost common words??")
# Show most common words
print(all_words.most_common(25))
# See how many times the word really is present in the reviews
#print("Presence of the word 'really':", all_words["Roma"])

def mostCommon(perc):
	number = (int) (len(all_words)*perc)
	return all_words.most_common(number)
# safe 0.8
word_feature_temp  = mostCommon(0.8)
word_feature = []
for h in word_feature_temp:
	word_feature.append(h[0])

#print word_feature

def find_feature(document):
	words_doc = word_tokenize(document)
	features = {}
	for w in word_feature:
		features[w] = (w in words_doc)
	return features

# Now let's create the featuresets for the Training Set:
featuresets = [(find_feature(rev),label) for (rev,label) in doc]
totFeat = len (featuresets)
print "Number of features TRAIN: ", totFeat
random.shuffle(featuresets)
#print featuresets[1]
#trSetPerc = 0.6

# Define Training and Test Set
#bound = (int) (totFeat*trSetPerc)
training_set = featuresets
#test_set = featuresets[bound+1:]

# Let's create the featuresets for the Test Set:
featuresets_test = [(find_feature(rev),label) for (rev,label) in docTest]
totFeat = len (featuresets_test)
print "Number of features TEST ", totFeat
random.shuffle(featuresets_test)
#print featuresets_test[1]

test_set = featuresets_test
 
###################################################################
##                     Train with Naive Bayes                    ##
###################################################################
if naiveB:
	classifier = nltk.NaiveBayesClassifier.train(training_set)
	#classifier_f = open("naiveBayes.pickle","rb")
	#classifier = pickle.load(classifier_f)
	#classifier_f.close()

	print(" Original Naive Bayes Alg acc:", (nltk.classify.accuracy(classifier,test_set))*100)
	classifier.show_most_informative_features(15)

###################################################################
##                         Train with SVM                        ##
###################################################################

if linSvm:
	#		Linear hyperplane speration
	SVC_classifier = SklearnClassifier(SVC(kernel='linear'))
	SVC_classifier.train(training_set)
	#classifier_f = open("naiveBayes.pickle","rb")
	#classifier = pickle.load(classifier_f)
	#classifier_f.close()

	print(" Support Vector Machine Classifier acc:", (nltk.classify.accuracy(SVC_classifier,test_set))*100)

#               Gaussian Hyperplane Separation

if gauSvm:
	SVCG_classifier = SklearnClassifier(SVC(kernel='rbf',gamma=0.0001,C=2.0))
	SVCG_classifier.train(training_set)
	#classifier_f = open("naiveBayes.pickle","rb")
	#classifier = pickle.load(classifier_f)
	#classifier_f.close()

	print(" Support Vector Machine Classifier acc:", (nltk.classify.accuracy(SVCG_classifier,test_set))*100)

###################################################################
##                         Train with SVM                        ##
###################################################################

if mnb:
	MNB_classifier = SklearnClassifier(MultinomialNB())
	MNB_classifier.train(training_set)
	print("Multinomial Classifier acc:", (nltk.classify.accuracy(MNB_classifier,test_set))*100)

	#save_classifier = open("MNBClassifier.pickle","wb")
	#pickle.dump(MNB_classifier,save_classifier)
	#save_classifier.close()

###################################################################
##                         Train with SVM                        ##
###################################################################

if lrc:
	LogisticRegression_classifier = SklearnClassifier(LogisticRegression())
	LogisticRegression_classifier.train(training_set)
	print("LogisticRegression Classifier acc:", (nltk.classify.accuracy(LogisticRegression_classifier,test_set))*100)

###################################################################
##                         Train with SVM                        ##
###################################################################

if sgd:
	SGD_classifier = SklearnClassifier(SGDClassifier(shuffle=True,alpha=0.00001,epsilon=0.1,learning_rate='optimal'))
	SGD_classifier.train(training_set)
	print("SGD Classifier acc:", (nltk.classify.accuracy(SGD_classifier,test_set))*100)

###################################################################
##                         Train with SVM                        ##
###################################################################

if lrc:
	LSVC_classifier = SklearnClassifier(LinearSVC())
	LSVC_classifier.train(training_set)
	print("Linear SVC Classifier acc:", (nltk.classify.accuracy(LSVC_classifier,test_set))*100)

if nuSvc:
	NuSVC_classifier = SklearnClassifier(NuSVC())
	NuSVC_classifier.train(training_set)
	print("Nu SVC Classifier acc:", (nltk.classify.accuracy(NuSVC_classifier,test_set))*100)

###################################################################
##                         Voting System                         ##
###################################################################

# Let's add a confidence parameter using a voting system. Next hack

#voted_classifier = VoteClassifier(MNB_classifier,
 #                                 LogisticRegression_classifier,
 #                                 LSVC_classifier,
 #                                 NuSVC_classifier)

#print("Voted classifier accuracy percent:",(nltk.classify.accuracy(voted_classifier,test_set))*100)


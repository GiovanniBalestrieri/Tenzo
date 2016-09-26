import io,csv,nltk,sys,random,pickle
reload(sys)
sys.setdefaultencoding('utf8')
from nltk.tokenize import word_tokenize
from nltk.corpus import stopwords

from nltk.classify.scikitlearn import SklearnClassifier
from sklearn.naive_bayes import MultinomialNB, GaussianNB ,BernoulliNB
from sklearn.linear_model import LogisticRegression, SGDClassifier
from sklearn.svm import SVC,LinearSVC,NuSVC
from nltk.classify import ClassifierI


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

# Pickle file

usepickle = False

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


posTrData = posTrData[:(int) (len(negTrData)*1.3)]
augmentedTitlesPos = []
augmentedTitlesNeg = []

augmentedTitlesPosTest = []
augmentedTitlesNegTest = []

for w in posTrData:
	#print "New Row: "
	w[0] = w[0] + " " +w[2] +" " +  w[1]
	#print w[0]
	augmentedTitlesPos.append(w[0].lower())

for w in negTrData:
	#print "New Row: "
	w[0] = w[0] + " " +w[2] +" " +  w[1]
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
sit.add("...")
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

#########################################################################		Preparing the Test Set
#######################################################################

print "\t\t\t\tTest Set initialization"
for w in posTtData:
	#print "New Row: "
	w[0] = w[0] + " " +w[2] +" " +  w[1]
	#print w[0]
	augmentedTitlesPosTest.append(w[0].lower())

for w in negTtData:
	#print "New Row: "
	w[0] = w[0] + " " +w[2] +" " +  w[1]
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

print "Removing duplicate entries\n -> Number of Test titles (+/-): ",len(augmentedTitlesPosTest)," ; ",len(augmentedTitlesNegTest)


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
safe = 0.9


word_feature_temp  = mostCommon(safe)
print "\n\nNumber of words in features considered in Train set: ", len(word_feature_temp)

# remove freq info from tuple
# actual ("word", freq ) post ("word")
word_feature = []
for h in word_feature_temp:
	word_feature.append(h[0])

#k = len(all_words)

#k1 = (int) (k*safe)
#word_feature =  list(word_feature_temp.keys())[:k1]
#print word_feature

def find_feature(document):
	words_doc = word_tokenize(document)
	word_feat = []
	for g in words_doc:
		word_feat.append(g.lower())
	features = {}
	for w in word_feature:
		features[w] = (w in word_feat)
	#print len(features)
	return features

# Now let's create the featuresets for the Training Set:
featuresets = [(find_feature(rev),label) for (rev,label) in doc]
totFeat = len (featuresets)
print "Number of features TRAIN: ", totFeat
random.shuffle(featuresets)
#print featuresets[0][1] #label

# Define Training and Test Set
training_set = featuresets

# Let's create the featuresets for the Test Set:
featuresets_test = [(find_feature(rev),label) for (rev,label) in docTest]
totFeat = len (featuresets_test)
print "Number of features TEST ", totFeat
random.shuffle(featuresets_test)
#print featuresets_test[1]
k = len(featuresets_test) 
test_set = featuresets_test[:(int) (safe*k)]
 
###################################################################
##                     Train with Naive Bayes                    ##
###################################################################
if naiveB:
	
	if usepickle:
		print "NO"
		#classifier_f = open("naiveBayes.pickle","rb")
		#classifier = pickle.load(classifier_f)
		#classifier_f.close()
	else:
		classifier = nltk.NaiveBayesClassifier.train(training_set)

	print(" Original Naive Bayes Alg acc:", (nltk.classify.accuracy(classifier,test_set))*100)
	classifier.show_most_informative_features(15)

	#save_classifier = open("naiveBayes.pickle","wb")
	#pickle.dump(classifier,save_classifier)
	#save_classifier.close()


###################################################################
##                         Train with SVM                        ##
###################################################################

if linSvm:

#		Linear hyperplane speration
	if usepickle:
                classifier_f = open("svmLin.pickle","rb")
                SVC_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:
		SVC_classifier = SklearnClassifier(SVC(kernel='linear'))
		SVC_classifier.train(training_set)

	print(" Support Vector Machine Linear acc:", (nltk.classify.accuracy(SVC_classifier,test_set))*100)
	
	save_classifier = open("svmLin.pickle","wb")
	pickle.dump(SVC_classifier,save_classifier)
	save_classifier.close()



if gauSvm:
	
#               Gaussian Hyperplane Separation
	if usepickle:
                classifier_f = open("svmG.pickle","rb")
                SVCG_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:		
		SVCG_classifier = SklearnClassifier(SVC(kernel='poly',C=1.0,degree=4))
		SVCG_classifier.train(training_set)

	print(" Support Vector Machine RBF acc:", (nltk.classify.accuracy(SVCG_classifier,test_set))*100)

	save_classifier = open("svmG.pickle","wb")
	pickle.dump(SVCG_classifier,save_classifier)
	save_classifier.close()


###################################################################
##                Train with Multinomial Classifier              ##
###################################################################

if mnb:
	if usepickle:
                classifier_f = open("MNB.pickle","rb")
                MNB_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:		
		MNB_classifier = SklearnClassifier(MultinomialNB())
		MNB_classifier.train(training_set)
	
	print("Multinomial Classifier acc:", (nltk.classify.accuracy(MNB_classifier,test_set))*100)

	save_classifier = open("MNB.pickle","wb")
	pickle.dump(MNB_classifier,save_classifier)
	save_classifier.close()



###################################################################
##                 Train with LogisticRegression                 ##
###################################################################

if lrc:
	if usepickle:
                classifier_f = open("lrc.pickle","rb")
                LogisticRegression_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:		
		LogisticRegression_classifier = SklearnClassifier(LogisticRegression())
		LogisticRegression_classifier.train(training_set)
	
	print("LogisticRegression Classifier acc:", (nltk.classify.accuracy(LogisticRegression_classifier,test_set))*100)

	save_classifier = open("lrc.pickle","wb")
	pickle.dump(LogisticRegression_classifier,save_classifier)
	save_classifier.close()



###################################################################
##                         Train with SGD                        ##
###################################################################

if sgd:
	if usepickle:
                classifier_f = open("sgd.pickle","rb")
                SGD_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:
		print "TRaining SVM"
		SGD_classifier = SklearnClassifier(SGDClassifier(shuffle=True,alpha=0.001,epsilon=0.1,learning_rate='optimal'))
		SGD_classifier.train(training_set)

	print("SGD Classifier acc:", (nltk.classify.accuracy(SGD_classifier,test_set))*100)
	
	save_classifier = open("sgd.pickle","wb")
	pickle.dump(SGD_classifier,save_classifier)
	save_classifier.close()


###################################################################
##                         Train with SVM                        ##
###################################################################

if lrc:
	if usepickle:
                classifier_f = open("LSVC.pickle","rb")
                LSVC_classifier = pickle.load(classifier_f)
                classifier_f.close()
        else:
		LSVC_classifier = SklearnClassifier(LinearSVC())
		LSVC_classifier.train(training_set)

	print("Linear SVC Classifier acc:", (nltk.classify.accuracy(LSVC_classifier,test_set))*100)

	save_classifier = open("LSVC.pickle","wb")
	pickle.dump(LSVC_classifier,save_classifier)
	save_classifier.close()

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

class VoteClassifier(ClassifierI):
        def __init__(self,*classifiers):
                self._classifiers = classifiers

        def classify(self,features):
                votes=[]
                for c in self._classifiers:
                        v = c.classify(features)
                        votes.append(v)

                modeVotes = max(set(votes), key=votes.count)
                return modeVotes
 
        def classifyString(self,string):
                features = find_feature(string)
		votes=[]
                for c in self._classifiers:
                        v = c.classify(features)
                        votes.append(v)

                modeVotes = max(set(votes), key=votes.count)
                return modeVotes

	def confidenceFromString(self,string):
		features = find_feature(string)	
                votes = []
                for c in self._classifiers:
                        v = c.classify(features)
			print "\n",v,"\nnext:"
                        votes.append(v)
                
                modeVotes = max(set(votes), key=votes.count)
                
                choice_votes = (float) (votes.count(modeVotes))
                tot = len(votes)
                conf = (float) (choice_votes/tot)
                #print(votes)
                #print(len(votes))
                #print("Confidence: ", conf)
                return conf


        def confidence(self,features):
                votes = []
                for c in self._classifiers:
                        v = c.classify(features)
                        votes.append(v)
                
                modeVotes = max(set(votes), key=votes.count)
                
                choice_votes = (float) (votes.count(modeVotes))
                tot = len(votes)
                conf = (float) (choice_votes/tot)
                #print(votes)
                #print(len(votes))
                #print("Confidence: ", conf)
                return conf

voted_classifier = VoteClassifier(MNB_classifier, 
                                  LogisticRegression_classifier,  
				  SVC_classifier,
				  classifier,
				  SGD_classifier,
				  SVCG_classifier,
                                  LSVC_classifier) 
 
print("Voted classifier accuracy percent:",(nltk.classify.accuracy(voted_classifier,test_set))*100) 

sentence0 = "Roma - cagliari"
sentence1 = "premio del ricchione"
sentence2 = "prix du ricchion"
sentence5 = "GRAN PREMIO DELLA LIBERAZIONE"
sentence3 = "La patata come la verdura "
sentence4 = "premio del gran gay"
sentence6 = "mangiare gay nel bosco delle fragole"

# Let's create the featuresets for the Test Set:
print("instance",sentence0, "Classification: ", voted_classifier.classifyString(sentence0), "Confidence: ", voted_classifier.confidenceFromString(sentence0)) 

print("instance",sentence1, "Classification: ", voted_classifier.classifyString(sentence1), "Confidence: ", voted_classifier.confidenceFromString(sentence1)) 
 
print("instance",sentence2, "Classification: ", voted_classifier.classifyString(sentence2), "Confidence: ", voted_classifier.confidenceFromString(sentence2)) 

print("instance",sentence3, "Classification: ", voted_classifier.classifyString(sentence3), "Confidence: ", voted_classifier.confidenceFromString(sentence3)) 

print("instance",sentence4, "Classification: ", voted_classifier.classifyString(sentence4), "Confidence: ", voted_classifier.confidenceFromString(sentence4)) 

print("instance",sentence5, "Classification: ", voted_classifier.classifyString(sentence5), "Confidence: ", voted_classifier.confidenceFromString(sentence5)) 

print("instance",sentence6, "Classification: ", voted_classifier.classifyString(sentence6), "Confidence: ", voted_classifier.confidenceFromString(sentence6)) 

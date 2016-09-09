from __future__ import division
import nltk
import random
from nltk.corpus import movie_reviews
import pickle
from nltk.classify.scikitlearn import SklearnClassifier
from sklearn.naive_bayes import MultinomialNB, GaussianNB ,BernoulliNB
from sklearn.linear_model import LogisticRegression, SGDClassifier
from sklearn.svm import SVC,LinearSVC,NuSVC

from nltk.classify import ClassifierI
from statistics import mode

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

	def confidence(self,features):
		votes = []
		for c in self._classifiers:
			v = c.classify(features)
			votes.append(v)
		
		modeVotes = max(set(votes), key=votes.count)
	        
		choice_votes = votes.count(modeVotes)
		tot = len(votes)
		conf = choice_votes/tot
		#print(votes)
		#print(len(votes))
		#print("Confidence: ", choice_votes)
		return conf


# List of tuples
documents=[(list(movie_reviews.words(fileid)),category)
	for category in movie_reviews.categories()
	for fileid in movie_reviews.fileids(category)]

#for category in movie_reviews.categories():
#	for fileid in movie_reviews.fileids(category):
#		documents.append(list(movie_reviews.words(fileid)),category)

random.shuffle(documents)

# Prints all the words in the reviews collection and its label pos/neg
#print(documents[2])

words = []
# Converto to lower case
for w in movie_reviews.words():
	words.append(w.lower())

# convert to nltk frequency distribution
all_words = nltk.FreqDist(words)
print("\t\t\tReally")
# Show most common words
print(all_words.most_common(15))
# See how many times the word stupid is present in the reviews
print(all_words["really"])

word_feature = list(all_words.keys())[:3000]

def find_feature(document):
	words = set(document)
	features = {}
	for w in word_feature:
		features[w] = (w in words)
	return features


#print(find_feature(movie_reviews.words('neg/cv000_29416.txt')))

featuresets = [(find_feature(rev),category) for (rev,category) in documents]

# Define Training and Test Set
training_set = featuresets[:1900]
test_set = featuresets[1900:]

# Train with Naive Bayes
#classifier = nltk.NaiveBayesClassifier.train(training_set)
classifier_f = open("naiveBayes.pickle","rb")
classifier = pickle.load(classifier_f)
classifier_f.close()

print(" Original Naive Bayes Alg acc:", (nltk.classify.accuracy(classifier,test_set))*100)
classifier.show_most_informative_features(15)

#save_classifier = open("naiveBayes.pickle","wb")
#pickle.dump(classifier,save_classifier)
#save_classifier.close()

# Uses a wrapper around nltk classfier
MNB_classifier = SklearnClassifier(MultinomialNB())
MNB_classifier.train(training_set)
print("Multinomial Classifier acc:", (nltk.classify.accuracy(MNB_classifier,test_set))*100)

save_classifier = open("MNBClassifier.pickle","wb")
pickle.dump(MNB_classifier,save_classifier)
save_classifier.close()

LogisticRegression_classifier = SklearnClassifier(LogisticRegression())
LogisticRegression_classifier.train(training_set)
print("LogisticRegression Classifier acc:", (nltk.classify.accuracy(LogisticRegression_classifier,test_set))*100)

save_classifier = open("logReg.pickle","wb")
pickle.dump(LogisticRegression_classifier,save_classifier)
save_classifier.close()

SGD_classifier = SklearnClassifier(SGDClassifier())
SGD_classifier.train(training_set)
print("SGD Classifier acc:", (nltk.classify.accuracy(SGD_classifier,test_set))*100)

#SVC_classifier = SklearnClassifier(SVC())
#SVC_classifier.train(training_set)
#print("SVC Classifier acc:", (nltk.classify.accuracy(SVC_classifier,test_set))*100)

#save_classifier = open("SVC.pickle","wb")
#pickle.dump(SVC_classifier,save_classifier)
#save_classifier.close()

LSVC_classifier = SklearnClassifier(LinearSVC())
LSVC_classifier.train(training_set)
print("Linear SVC Classifier acc:", (nltk.classify.accuracy(LSVC_classifier,test_set))*100)

NuSVC_classifier = SklearnClassifier(NuSVC())
NuSVC_classifier.train(training_set)
print("Nu SVC Classifier acc:", (nltk.classify.accuracy(NuSVC_classifier,test_set))*100)


# Let's add a confidence parameter using a voting system. Next hack

voted_classifier = VoteClassifier(MNB_classifier,
				  LogisticRegression_classifier, 
				  LSVC_classifier,
				  NuSVC_classifier)

print("Voted classifier accuracy percent:",(nltk.classify.accuracy(voted_classifier,test_set))*100)

print("Classification: ", voted_classifier.classify(test_set[0][0]), "Confidence: ", voted_classifier.confidence(test_set[0][0]))

print("Classification: ", voted_classifier.classify(test_set[1][0]), "Confidence: ", voted_classifier.confidence(test_set[1][0]))

print("Classification: ", voted_classifier.classify(test_set[2][0]), "Confidence: ", voted_classifier.confidence(test_set[2][0]))

print("Classification: ", voted_classifier.classify(test_set[4][0]), "Confidence: ", voted_classifier.confidence(test_set[3][0]))

print("Classification: ", voted_classifier.classify(test_set[4][0]), "Confidence: ", voted_classifier.confidence(test_set[4][0]))

from __future__ import division
import nltk,io,random,pickle
from nltk.corpus import movie_reviews
from nltk.classify.scikitlearn import SklearnClassifier
from sklearn.naive_bayes import MultinomialNB, GaussianNB ,BernoulliNB
from sklearn.linear_model import LogisticRegression, SGDClassifier
from sklearn.svm import SVC,LinearSVC,NuSVC
from nltk.corpus import stopwords
from nltk.classify import ClassifierI
from statistics import mode
from nltk.tokenize import word_tokenize

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

short_pos = io.open("pos.txt","r",encoding='utf-8').read()
short_neg = io.open("neg.txt","r",encoding='utf-8').read()

documents = []

for r in short_pos.split('\n'):
	documents.append((r,"pos"))
for r in short_neg.split('\n'):
	documents.append((r,"neg"))

all_words = []

short_pos_words = word_tokenize(short_pos)
short_neg_words = word_tokenize(short_neg)

for w in short_pos_words:
	all_words.append(w.lower())
for w in short_neg_words:
	all_words.append(w.lower()) 

ss=set(stopwords.words('english'))
#print("Stopwords in English: ",ss)
ss.add(",")
ss.add(".")
s.add("'s")
ss.add("'")
#ss.remove('not') # Meglio senza not, confonde
all_clean = []

for word in all_words:
	if word not in ss:
		all_clean.append(word)

all_words = all_clean
#print(all_words)

# convert to nltk frequency distribution
all_words = nltk.FreqDist(all_words)
print("\t\t\tReally")
# Show most common words
print(all_words.most_common(15))
# See how many times the word really is present in the reviews
print("Presence of the word 'really':", all_words["really"])


print(len(all_words))
word_feature = list(all_words.keys())[:4000]

def find_feature(document):
	#words = set(document)
	words = word_tokenize(document)
	features = {}
	for w in word_feature:
		features[w] = (w in words)
	return features


#print(find_feature(movie_reviews.words('neg/cv000_29416.txt')))

featuresets = [(find_feature(rev),category) for (rev,category) in documents]

print("Number of features: ",len(featuresets)) 
random.shuffle(featuresets)

# Define Training and Test Set
training_set = featuresets[:6000]
test_set = featuresets[6001:]

# Train with Naive Bayes
classifier = nltk.NaiveBayesClassifier.train(training_set)
#classifier_f = open("naiveBayes.pickle","rb")
#classifier = pickle.load(classifier_f)
#classifier_f.close()

print(" Original Naive Bayes Alg acc:", (nltk.classify.accuracy(classifier,test_set))*100)
classifier.show_most_informative_features(15)

save_classifier = open("naiveBayes.pickle","wb")
pickle.dump(classifier,save_classifier)
save_classifier.close()

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

SVC_classifier = SklearnClassifier(SVC(kernel='linear',gamma='0.01'))
SVC_classifier.train(training_set)
print("SVC Classifier acc:", (nltk.classify.accuracy(SVC_classifier,test_set))*100)

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

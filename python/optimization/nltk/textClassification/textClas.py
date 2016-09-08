import nltk
import random
from nltk.corpus import movie_reviews
import pickle
from nltk.classify.scikitlearn import SklearnClassifier
from sklearn.naive_bayes import MultinomialNB, GaussianNB ,BernoulliNB

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

word_feature = list(all_words.keys())[:5000]

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
classifier = nltk.NaiveBayesClassifier.train(training_set)
#classifer_f = open("naiveBayes.pickle","rb")
#classfier = pickle.load(classifier_f)
#classifier_f.close()

print(" Original Naive Bayes Alg acc:", (nltk.classify.accuracy(classifier,test_set))*100)
classifier.show_most_informative_features(15)

save_classifier = open("naiveBayes.pickle","wb")
pickle.dump(classifier,save_classifier)
save_classifier.close()

# Uses a wrapper around nltk classfier
MNB_classifier = SklearnClassifier(MultinomialNB())
MNB_classifier.train(training_set)
print(" Original Multinomial Classifier acc:", (nltk.classify.accuracy(MNB_classifier,test_set))*100)


#GuassianNB_classifier = SklearnClassifier(GaussianNB())
#GaussianMNB_classifier.train(training_set)
#print(" Original Gaussian classifier acc:", (nltk.classify.accuracy(GaussianMNB_classifier,test_set))*100)



from nltk.tokenize import sent_tokenize,word_tokenize
import io

sentence = "ok how are you. I am fine. Now where are you heading? how is your mom?"

sentence1 = "Hello, Mr. Smith, how was your day?"
short_neg = io.open("../textClassification/neg.txt","r",encoding='utf-8').read()

documents = []

for r in short_neg.split('\n'):
        documents.append((r,"neg"))

all_words = []

short_pos_words = word_tokenize(short_neg)


# Recognize sentences
print(sent_tokenize(sentence))

#Recognize words. 
print(word_tokenize(sentence))

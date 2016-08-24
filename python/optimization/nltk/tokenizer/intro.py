from nltk.tokenize import sent_tokenize,word_tokenize


sentence = "ok how are you. I am fine. Now where are you heading? how is your mom?"

sentence1 = "Hello, Mr. Smith, how was your day?"

# Recognize sentences
print(sent_tokenize(sentence))

#Recognize words. 
print(word_tokenize(sentence))

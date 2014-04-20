"""
A module for interfacing with the Stanford Pparser.
"""

import os
from subprocess import PIPE
import tempfile
import nltk
import sys

# try:
#     from pathscript import *
# except ImportError:
#     print "PATHSCRIPT COULDN'T BE IMPORTED!!"
#     print "pathscript.py isn't in your computer. \n please, create it in: \n $roscd gpsrSoar/src/"
#     print "then define: \nPATH_TO_SOAR=\nPATH_TO_STANFORD_PARSER=\n\npointing to these packages folders"
# import roslib
# from api import *

from pathscript import *
sys.path.append(PATH_TO_STANFORD_PARSER)

_stanford_url = 'http://nlp.stanford.edu/software/tagger.shtml'


class StanfordParser(object):
    def __init__(self, path_to_model, path_to_jar=None,
        encoding=None, verbose=False):

        # if path_to_jar == None:
        # PATH_TO_STANFORD_PARSER = '/home/jy/reem_at_iri/stanford-parser-2013-04-05/'
        path_to_jar = PATH_TO_STANFORD_PARSER + 'stanford-parser.jar' #PATH_TO_STANFORD_PARSER #+ "stanford-parser.jar"

        self._stanford_jar = nltk.internals.find_jar(
            'stanford-parser.jar', path_to_jar,
            searchpath=(), url=_stanford_url,
            verbose=verbose)

        path_to_model = PATH_TO_STANFORD_PARSER + 'models/lexparser/englishPCFG.ser.gz'

        if not os.path.isfile(path_to_model):
            raise IOError("Stanford parser model file not found: %s" % path_to_model)
        self._stanford_model = path_to_model
        self._encoding = encoding

    def batch_parse(self, sentences):
        encoding = self._encoding
        nltk.internals.config_java(options='-mx1000m', verbose=False)

        # Create a temporary input file
        _input_fh, _input_file_path = tempfile.mkstemp(text=True)

        # Build the java command to run the tagger
        _stanpos_cmd = [
                    'edu.stanford.nlp.parser.lexparser.LexicalizedParser', \
                    '-outputFormat', "penn,typedDependencies", \
                    '-tokenized', \
                    '-tagSeparator', '/', \
                    '-tokenizerFactory', 'edu.stanford.nlp.process.WhitespaceTokenizer', \
                    '-tokenizerMethod', 'newCoreLabelTokenizerFactory', \
                    self._stanford_model, \
                    _input_file_path]
        if encoding:
            _stanpos_cmd.extend(['-encoding', encoding])
        # _stanpos_cmd.extend(['-tokenized'])
        # _stanpos_cmd.extend(['-tagSeparator', '/'])
        # _stanpos_cmd.extend(['-tokenizerFactory', 'edu.stanford.nlp.process.WhitespaceTokenizer'])
        # _stanpos_cmd.extend(['-tokenizerMethod', 'newCoreLabelTokenizerFactory'])
        # _stanpos_cmd.extend([_input_file_path])
        # _stanpos_cmd.extend([self._stanford_model])

        sentences=[nltk.word_tokenize(sentences)]


        # Write the actual sentences to the temporary input file
        _input_fh = os.fdopen(_input_fh, 'w')
        _input = '\n'.join((' '.join(x) for x in sentences))
        # _input = (' '.join(x) for x in sentences)
        if isinstance(_input, unicode) and encoding:
            _input = _input.encode(encoding)
        _input_fh.write(_input)
        _input_fh.close()

        # Run the tagger and get the output
        stanpos_output, _stderr = nltk.internals.java(
                                            _stanpos_cmd, \
                                            classpath=self._stanford_jar, \
                                            stdout=PIPE, stderr=PIPE)
        if encoding:
            stanpos_output = stanpos_output.decode(encoding)

        # Delete the temporary file
        os.unlink(_input_file_path)
        print stanpos_output

        return stanpos_output


        # Output the parsed sentences
        # parsed_sentences = []
        # for parsed_sentence in stanpos_output.strip().split("\n"):





        # parsed_sentences = []
        # for parsed_sentence in stanpos_output.strip().split("\n"):
        #     sentence = [tuple(tagged_word.strip().split("_"))
        #                 for tagged_word in parsed_sentence.strip().split()]
        #     parsed_sentences.append(sentence)
        # return parsed_sentences

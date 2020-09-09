from owlready2 import *
"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manage the ontology or KnowledgeBase (KB)
"""

# Returns
# - tbox: the ontology containing the Tbox frmo the tomasys.owl
# - abox: the ontology containing the individuals to initialize the KB, aka the abox 
def loadTomasysKB(tboxfile, abox_file):
    onto_path.append(os.path.dirname(os.path.realpath(tboxfile)))
    onto_path.append(os.path.dirname(os.path.realpath(abox_file))) 
    global tomasys, onto
    tbox = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file (that does not seem to work)
    abox = get_ontology(abox_file).load()
    return tbox, abox



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

# Given
# - o: individual of tomasys:Objective
# - abox: ontology that contains the individuals in the KB
# - tomasys ontology that contains the tomasys tbox
def obtainBestFunctionDesign(o, abox, tomasys):
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tomasys.FunctionDesign.instances()):
        if fd.solvesF == f:
            fds.append(fd)
    print("== FunctionDesigns available for obj: %s", str([fd.name for fd in fds]))
    print("Objective NFR ENERGY: %s", str(o.hasNFR))

    # fiter fds to only those available
    # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
    realisable_fds = [fd for fd in fds if fd.fd_realisability != False]
    # print("== FunctionDesigns REALISABLE for obj: ", [fd.name for fd in realisable_fds])
    # discard FDs already grounded for this objective when objective in error
    suitable_fds= [fd for fd in fds if (not o in fd.fd_error_log)]
    # print("== FunctionDesigns suitable NOT IN ERROR LOG: ", [fd.name for fd in suitable_fds])
    # discard those FD that will not meet objective NFRs
    fds_for_obj = meetNFRs(o, suitable_fds)
    # get best FD based on higher Utility/trade-off of QAs
    if fds_for_obj != []:
        print("== FunctionDesigns also meeting NFRs: %s", [fd.name for fd in fds_for_obj])
        aux = 0
        best_fd = fds_for_obj[0]
        for fd in fds_for_obj:
            u = utility(fd)
            if  u > aux:
                best_fd = fd
                aux = u

        print("> Best FD available %s", str(best_fd.name))
        return best_fd
    else:
        print("*** OPERATOR NEEDED, NO SOLUTION FOUND ***")
        return None

# Returns all FunctionDesign individuals from a given set (fds) that comply with the NFRs of a giben Objective individual (o)
def meetNFRs(o, fds):
    if fds == []:
        print("Empty set of given FDs")
        return []
    filtered = []
    if len(o.hasNFR) == 0:
        print("== Objective has no NFRs, so a random FD is picked")
        return [next(iter(fds))]
    print("== Checking FDs for Objective with NFRs type: %s and value %s ", str(o.hasNFR[0].isQAtype.name), str(o.hasNFR[0].hasValue))
    for fd in fds:
        for nfr in o.hasNFR:
            qas = [qa for qa in fd.hasQAestimation if qa.isQAtype is nfr.isQAtype]
        if len(qas) != 1:
            print("FD has no expected value for this QA or multiple definitions (inconsistent)")
            break
        else:
            if nfr.isQAtype.name == 'energy':
                if qas[0].hasValue > nfr.hasValue: # specific semantics for energy
                    break
            elif nfr.isQAtype.name == 'safety':
                if qas[0].hasValue < nfr.hasValue:  # specific semantics for energy
                    break
            else:
                print("No known criteria for FD selection for that QA")
        filtered.append(fd)
    if filtered == []:
        print("No FDs meetf NFRs")

    return filtered

# Compute expected utility based on QA trade-off, the criteria to chose FDs/configurations
# TODO utility is the selection criteria for FDs and it is hardcoded as QA performance
def utility(fd):
    # utility is equal to the expected time performance
    utility = [
        qa.hasValue for qa in fd.hasQAestimation if "perfomance" in qa.isQAtype.name]
    if len(utility) == 0:
        return 0.001    #if utility is not known it is assumed to be 0.001 (very low)
    else:
        return utility[0]
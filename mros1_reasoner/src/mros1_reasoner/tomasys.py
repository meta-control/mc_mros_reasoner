from owlready2 import *
###########################################
# 
# authors:  c.h.corbato@tudelft.nl
#           M.A.GarzonOviedo@tudelft.nl
#
# DESCRIPTION:
#  Python library implementing utilities to manipulate Knowledge Bases
#  based on the tomasys metamodel (Tbox), using the owlready2 library
##########################################


# Returns
# - kb_box: the ontology readed
def loadKB_from_file(kb_file):
    """ Reads a KB from a given file
        (Replaces loadTomasysKB)
        Args:
                kb_file (string): Full path to the ontology to be loaded.
        Returns:
                kb_box (ontology): Ontology readed, None: if there is an error.
    """
    try:
        kb_box = get_ontology(kb_file).load()
    except Exception as e:
        return None
    return kb_box
# # Returns
# # - tbox: the ontology containing the Tbox frmo the tomasys.owl
# # - abox: the ontology containing the individuals to initialize the KB, aka the abox
# def loadTomasysKB(tboxfile, abox_file):
#     onto_path.append(os.path.dirname(os.path.realpath(tboxfile)))
#     onto_path.append(os.path.dirname(os.path.realpath(abox_file)))
#     tbox = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file (that does not seem to work)
#     abox = get_ontology(abox_file).load()
#     return tbox, abox

# To reset the individuals that no longer hold due to adaptation
# for the moment, only Objective individuals statuses
# - tomasys: ontology holding the Tbox
def resetKBstatuses(tbox):
    for o in list(tbox.Objective.instances()):
        o.o_status = None

# For debugging purposes
def print_ontology_status(kb_box):
    # print("\nComponents Statuses:")
    # for i in list(tomasys.ComponentState.instances()):
    #     print(i.name, i.c_status)

    # print("\nBindings Statuses:")
    # for i in list(tomasys.Binding.instances()):
    #     print(i.name, i.b_status)

    print("\nFGs:")
    for i in list(kb_box.FunctionGrounding.instances()):
        print(i.name, "\tobjective: ", i.solvesO, "\tstatus: ", i.fg_status, "\tFD: ",
        i.typeFD, "\tQAvalues: ", [(qa.isQAtype.name, qa.hasValue) for qa in i.hasQAvalue])

    print("\nOBJECTIVE\t|  STATUS\t|  NFRs")
    for i in list(kb_box.Objective.instances()):
        print(i.name,"\t|  ", i.o_status, "\t|  ", [(nfr.isQAtype.name, nfr.hasValue) for nfr in i.hasNFR])

    # print("\nCC availability:")
    # for i in list(tomasys.ComponentClass.instances()):
    #     print(i.name, i.cc_availability)

    # print("\nFDs information:\n NAME \t")
    # for i in list(tomasys.FunctionDesign.instances()):
    #     print(i.name, "\t", i.fd_realisability, "\t", [
    #           (qa.isQAtype.name, qa.hasValue) for qa in i.hasQAestimation])

# update the QA value for an FG with the value received
def updateQAvalue(fg, qa_type, value, tbox, abox):
    qas = fg.hasQAvalue

    if qas == []: # for the first qa value received
        qav = tbox.QAvalue("obs_{}".format(qa_type.name
                                               ), namespace=abox, isQAtype=qa_type, hasValue=value)
        fg.hasQAvalue.append(qav)
    else:
        for qa in qas:
            if qa.isQAtype == qa_type:
                qa.hasValue = value
                return
        # case it is a new QA type value
        qav = tbox.QAvalue("obs_{}".format(qa_type.name
                                         ), isQAtype=qa_type, namespace=abox, hasValue=value)
        fg.hasQAvalue.append(qav)

# Evaluates the Objective individuals in the KB and returns a list with those in error
def evaluateObjectives(tbox):
    objectives_internal_error = []
    for o in list(tbox.Objective.instances() ):
        if o.o_status == "INTERNAL_ERROR":
            objectives_internal_error.append(o)
    return objectives_internal_error


# Select best FD in the KB, given:
# - o: individual of tomasys:Objective
# - tomasys ontology that contains the tomasys tbox
def obtainBestFunctionDesign(o, tbox):
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tbox.FunctionDesign.instances()):
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


def ground_fd(fd, objective, tbox, abox):
    """Given a FunctionDesign fd and an Objective objective, creates an individual FunctionGrounds with typeF fd and solve) objective
        returns the fg
    """
    fg = tbox.FunctionGrounding(
        "fg_"+fd.name.replace('fd_', ''), namespace=abox, typeFD=fd, solvesO=objective)
    # TODO: ground objectives required by FD
    return fg

def remove_objective_grounding(objective, tbox, abox):
    """Given an objective individual, removes the grounded hierarchy (fg tree) that solves it.
    """
    fg = abox.search_one(solvesO=objective)
    if fg:
        destroy_entity(fg)


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

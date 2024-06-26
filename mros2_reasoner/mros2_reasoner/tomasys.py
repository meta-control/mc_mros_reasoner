###########################################
#
# authors:  c.h.corbato@tudelft.nl
#           M.A.GarzonOviedo@tudelft.nl
#
# DESCRIPTION:
#  Python library implementing utilities to manipulate Knowledge Bases
#  based on the tomasys metamodel (Tbox), using the owlready2 library
##########################################

from owlready2 import destroy_entity
from owlready2 import get_ontology
from typing import List
from typing import Tuple
import logging


# Returns
# - kb_box: the ontology readed
def load_kb_from_file(kb_file):
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
        logging.exception("{0}".format(e))
        return None
    return kb_box


def read_ontology_file(ontology_file_array):
    """ Checks if an ontology file exists and reads its value
        Args:
                ontology_file_name (array string): The name of the parameter.
        Returns:
                The ontology if it's readed correctly, None otherwise.
    """
    if (type(ontology_file_array) is str):
        ontology_file_array = [ontology_file_array]
    ontology_obj = None
    for ontology_file in ontology_file_array:
        if ontology_file is not None:
            ontology = load_kb_from_file(ontology_file)
            if ontology is not None:
                logging.info("Loaded ontology: " + str(ontology_file))
            else:
                logging.error(
                    "Failed to load ontology from: " +
                    str(ontology_file))
                return None
            if ontology_obj:
                ontology_obj.imported_ontologies.append(ontology)
            else:
                ontology_obj = ontology

    if ontology_obj is None:
        logging.error("Error while reading ontology files!")
        return None
    return ontology_obj


# To reset the individuals that no longer hold due to adaptation
# for the moment, only Objective individuals statuses
# - ontology: ontology holding the Tbox
def reset_objective_status(objective_name: str, ontology, status=None):
    # logging.warning("\nReseting obj {0}".format(objective.name))
    objective = ontology.search_one(
        iri="*{}".format(objective_name), is_a=ontology.Objective)
    if objective is not None:
        objective.o_status = status
        objective.o_updatable = None


def reset_fd_realisability(ontology, c_name):
    logging.warning("\nReset realisability:\n")
    component = ontology.search_one(iri="*{}".format(c_name))
    if component is None:
        # logging.warning("C not found Return\n\n\n")
        return

    if component.c_status is None:
        # logging.warning("C status None Return\n\n\n")
        return
    else:
        if component.c_status in ["FALSE", "RECOVERED"]:
            logging.warning(
                "component status is {} - Set to None\n".format(
                    component.c_status))
            for fd in list(ontology.FunctionDesign.instances()):
                if fd.fd_realisability is None:
                    continue
                else:
                    logging.warning(
                        "FD {0} realisability: {1} -  Set to None".format(
                            fd.name, fd.fd_realisability))
                    fd.fd_realisability = None
            component.c_status = None


# For debugging purposes
def print_ontology_status(kb_box):
    logging.warning("\t\t\t >>> Ontology Status   <<<")

    logging.warning("\n\tComponent Status:\t{0}".format(
        [(c.name, c.c_status)
            for c in list(kb_box.ComponentState.instances())]))

    for i in list(kb_box.FunctionGrounding.instances()):
        logging.warning(
            "\n\tFG: {0}  Status: {1}  Solves: {2}  FD: {3}  QAvalues: {4}"
            .format(
                i.name, i.fg_status, i.solvesO.name, i.typeFD.name, [
                    (qa.isQAtype.name, qa.hasValue) for qa in i.hasQAvalue]))

    for i in list(kb_box.Objective.instances()):
        logging.warning("\n\tOBJECTIVE: {0}   Status: {1}   NFRs:  {2}".format(
            i.name,
            i.o_status,
            [(nfr.isQAtype.name, nfr.hasValue) for nfr in i.hasNFR]))
    logging.warning("\t\t\t >>>>>>>>>>>>> <<<<<<<<<<<")


# update the QA value for an FG with the value received
def update_measured_qa_value(qa_type, value, ontology):
    measured_qa = ontology.QAvalue(
        "obs_{}".format(
            qa_type.name),
        namespace=ontology,
        isQAtype=qa_type,
        hasValue=value)
    return measured_qa


def update_fg_measured_qa(fg, measured_qa):
    updated = False
    for qa in fg.hasQAvalue:
        if str(qa.name) == str(measured_qa.name):
            qa = measured_qa
            updated = True
    if not updated:
        fg.hasQAvalue.append(measured_qa)


# Evaluates the Objective individuals in the KB and returns a list with
# those in error
def get_adaptable_objectives(objectives) -> Tuple[List[str], List[str]]:
    objectives_adaptable = []
    objectives_adaptable_status = []
    o_error_status = ["UNGROUNDED",
                      "IN_ERROR_FR",
                      "IN_ERROR_NFR",
                      "IN_ERROR_COMPONENT"]
    for o in objectives:
        if o.o_status in o_error_status or o.o_updatable is True:
            objectives_adaptable.append(str(o.name))
            objectives_adaptable_status.append(str(o.o_status))
        elif o.o_always_improve is True:
            objectives_adaptable.append(str(o.name))
            objectives_adaptable_status.append(str(o.o_status))
    return objectives_adaptable, objectives_adaptable_status


def get_function_grounding(o, ontology):
    fgs = ontology.FunctionGrounding.instances()
    for fg in fgs:
        if fg.solvesO == o:
            return fg
    return None


def get_current_function_design(o, ontology):
    fg = get_function_grounding(o, ontology)
    if fg is not None:
        return fg.typeFD
    return None


# Select best FD in the KB, given:
# - o: individual of tomasys:Objective
# - tomasys ontology that contains the tomasys ontology
def obtain_best_function_design(o, ontology):
    logging.warning("\t\t\t == Obatin Best Function Design ==")
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(ontology.FunctionDesign.instances()):
        if fd.solvesF == f:
            fds.append(fd)
    logging.warning("== FunctionDesigns AVAILABLE: %s",
                    str([fd.name for fd in fds]))

    # fiter fds to only those available
    # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are
    # complete for this)
    realisable_fds = [fd for fd in fds if fd.fd_realisability is not False]
    logging.warning("== FunctionDesigns REALISABLE: %s",
                    str([fd.name for fd in realisable_fds]))
    # discard FDs already grounded for this objective when objective in error
    suitable_fds = [
        fd for fd in fds if (
            (o not in fd.fd_error_log) and (
                fd.fd_realisability is not False))]
    logging.warning("== FunctionDesigns NOT IN ERROR LOG: %s",
                    str([fd.name for fd in suitable_fds]))
    # discard those FD that will not meet objective NFRs

    fds_for_obj = filter_fds(o, suitable_fds, ontology)
    if fds_for_obj != []:
        best_utility = 0
        for fd in fds_for_obj:
            # if fd != current_fd:
            utility_fd = utility(fd)
            logging.warning("== Utility for %s : %f", fd.name, utility_fd)
            if utility_fd > best_utility:
                best_fd = fd
                best_utility = utility_fd

        logging.warning("\t\t\t == Best FD available %s", str(best_fd.name))
        return best_fd.name
    else:
        logging.warning("\t\t\t == *** NO SOLUTION FOUND ***")
        return None


def ground_fd(fd_name: str, objective_name: str, ontology):
    """Given a FunctionDesign fd and an Objective objective, creates an
       individual FunctionGrounds with typeF fd and solve) objective returns
       the fg
    """
    objective = ontology.search_one(
        iri="*{}".format(objective_name), is_a=ontology.Objective)
    fd = ontology.search_one(
        iri="*{}".format(fd_name), is_a=ontology.FunctionDesign)
    if objective is not None and fd is not None:
        fg = ontology.FunctionGrounding(
            "fg_" +
            fd.name.replace(
                'fd_',
                ''),
            namespace=ontology,
            typeFD=fd,
            solvesO=objective)
        # TODO: ground objectives required by FD
        return fg


def remove_objective_grounding(objective_name, ontology):
    """Given an objective individual, removes the grounded hierarchy (fg tree)
        that solves it.
    """
    objective = ontology.search_one(iri="*{}".format(objective_name))
    if objective is not None:
        fg = ontology.search_one(solvesO=objective)
        if fg is not None:
            destroy_entity(fg)


def get_measured_qa(key, ontology):
    observed_qa_value = None
    # TODO: is it better to use instances()?
    qa_values = ontology.search(type=ontology.QAvalue)
    for qa in qa_values:
        if qa.name == 'obs_' + key:
            observed_qa_value = qa.hasValue
            break
    return observed_qa_value


def filter_fds(o, fds, ontology):
    filtered = meet_objective_nfrs(o, fds)
    logging.warning(
        "== FunctionDesigns also meeting objectives NFRs: %s", [
            fd.name for fd in filtered])
    filtered = filter_fds_nfr(o, fds, ontology)
    logging.warning(
        "== FunctionDesigns also meeting fds NFRs: %s", [
            fd.name for fd in filtered])
    return filtered


# Returns all FunctionDesign individuals from a given set (fds) that
# comply with the NFRs of a given Objective individual (o)
def meet_objective_nfrs(o, fds):
    if fds == [] or len(o.hasNFR) == 0:
        return fds

    filtered = []
    for fd in fds:
        for nfr in o.hasNFR:
            qas = [qa for qa in fd.hasQAestimation
                   if str(qa.isQAtype) == str(nfr.isQAtype)]
        if len(qas) != 1:
            logging.warning(
                'FD has no expected value for this QA or multiple definitions')
            break
        else:
            if nfr.isQAtype.name == 'energy':
                if qas[0].hasValue > nfr.hasValue:
                    break
            elif nfr.isQAtype.name == 'safety':
                if qas[0].hasValue < nfr.hasValue:
                    break
        filtered.append(fd)
    if filtered == []:
        logging.warning("No FDs meet NFRs")

    return filtered


def filter_fds_nfr(o, fds, ontology):
    filtered = fds.copy()
    for fd in fds:
        for qa in fd.hasQAestimation:
            qa_type = qa.isQAtype
            operator = qa_type.qa_comparison_operator
            measured_qa = get_measured_qa(qa_type.name, ontology)
            if operator == "<" and measured_qa < qa.hasValue:
                filtered.remove(fd)
            elif operator == "<=" and measured_qa <= qa.hasValue:
                filtered.remove(fd)
            elif operator == ">" and measured_qa > qa.hasValue:
                filtered.remove(fd)
            elif operator == ">=" and measured_qa >= qa.hasValue:
                filtered.remove(fd)
    return filtered


# Compute expected utility based on QA trade-off, the criteria to chose FDs
# TODO utility is the selection criteria for FDs and it is hardcoded as QA
# performance
def utility(fd):
    # utility is equal to the expected performance
    utility = [
        qa for qa in fd.hasQAestimation if qa.isQAtype.name == "performance"]
    if len(utility) != 1:
        logging.warning(
            "FD has no expected perfomance or multiple definitions")
        return 0.001
    else:
        return utility[0].hasValue

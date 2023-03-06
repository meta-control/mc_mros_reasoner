from mros2_reasoner.kb_interface import KBInterface


class TOMASysInterface(KBInterface):
    def __init__(self):
        self.knowledge_base = None

    def get_tasks(self):
        pass

    def get_task(self, task_name):
        pass

    def get_functions(self):
        pass

    def get_function(self, function_name):
        pass

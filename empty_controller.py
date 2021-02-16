class TulipStrategy(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2021-02-15 18:44:16 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 3
        self.input_vars = ['xempty']

    def move(self, xempty):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['xcar', 'vcar']
        """
        output = dict()
        if self.state == 0:
            if (xempty == 0):
                self.state = 1

                output["xcar"] = 2
                output["vcar"] = 0
            elif (xempty == 1):
                self.state = 2

                output["xcar"] = 2
                output["vcar"] = 0
            else:
                self._error(xempty)
        elif self.state == 1:
            if (xempty == 0):
                self.state = 1

                output["xcar"] = 2
                output["vcar"] = 0
            elif (xempty == 1):
                self.state = 2

                output["xcar"] = 2
                output["vcar"] = 0
            else:
                self._error(xempty)
        elif self.state == 2:
            if (xempty == 0):
                self.state = 1

                output["xcar"] = 2
                output["vcar"] = 0
            elif (xempty == 1):
                self.state = 2

                output["xcar"] = 2
                output["vcar"] = 0
            else:
                self._error(xempty)
        elif self.state == 3:
            if (xempty == 1):
                self.state = 0

                output["xcar"] = 1
                output["vcar"] = 1
            else:
                self._error(xempty)
        else:
            raise Exception("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, xempty):
        raise ValueError("Unrecognized input: " + (
            "xempty = {xempty}; ").format(
                xempty=xempty))

class TulipStrategy(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2021-03-01 00:24:13 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 8
        self.input_vars = ['xempty']

    def move(self, xempty):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['xcar', 'vcar']
        """
        output = dict()
        if self.state == 0:
            if (xempty == 1):
                self.state = 1

                output["xcar"] = 11
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 1:
            if (xempty == 1):
                self.state = 2

                output["xcar"] = 21
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 2:
            if (xempty == 1):
                self.state = 3

                output["xcar"] = 31
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 3:
            if (xempty == 1):
                self.state = 4

                output["xcar"] = 41
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 4:
            if (xempty == 1):
                self.state = 5

                output["xcar"] = 51
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 5:
            if (xempty == 1):
                self.state = 6

                output["xcar"] = 61
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 6:
            if (xempty == 1):
                self.state = 7

                output["xcar"] = 65
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 7:
            if (xempty == 1):
                self.state = 7

                output["xcar"] = 65
                output["vcar"] = 10
            else:
                self._error(xempty)
        elif self.state == 8:
            if (xempty == 1):
                self.state = 0

                output["xcar"] = 1
                output["vcar"] = 10
            else:
                self._error(xempty)
        else:
            raise Exception("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, xempty):
        raise ValueError("Unrecognized input: " + (
            "xempty = {xempty}; ").format(
                xempty=xempty))

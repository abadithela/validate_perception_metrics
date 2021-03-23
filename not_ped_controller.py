class TulipStrategy(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2021-03-23 20:01:44 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 20
        self.input_vars = ['xobj']

    def move(self, xobj):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['xcar', 'vcar']
        """
        output = dict()
        if self.state == 0:
            if (xobj == 1):
                self.state = 1

                output["xcar"] = 11
                output["vcar"] = 9
            else:
                self._error(xobj)
        elif self.state == 1:
            if (xobj == 1):
                self.state = 2

                output["xcar"] = 20
                output["vcar"] = 8
            else:
                self._error(xobj)
        elif self.state == 2:
            if (xobj == 1):
                self.state = 3

                output["xcar"] = 28
                output["vcar"] = 7
            else:
                self._error(xobj)
        elif self.state == 3:
            if (xobj == 1):
                self.state = 4

                output["xcar"] = 35
                output["vcar"] = 6
            else:
                self._error(xobj)
        elif self.state == 4:
            if (xobj == 1):
                self.state = 5

                output["xcar"] = 41
                output["vcar"] = 5
            else:
                self._error(xobj)
        elif self.state == 5:
            if (xobj == 1):
                self.state = 6

                output["xcar"] = 46
                output["vcar"] = 4
            else:
                self._error(xobj)
        elif self.state == 6:
            if (xobj == 1):
                self.state = 7

                output["xcar"] = 50
                output["vcar"] = 3
            else:
                self._error(xobj)
        elif self.state == 7:
            if (xobj == 1):
                self.state = 8

                output["xcar"] = 53
                output["vcar"] = 2
            else:
                self._error(xobj)
        elif self.state == 8:
            if (xobj == 1):
                self.state = 9

                output["xcar"] = 55
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 9:
            if (xobj == 1):
                self.state = 10

                output["xcar"] = 56
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 10:
            if (xobj == 1):
                self.state = 11

                output["xcar"] = 57
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 11:
            if (xobj == 1):
                self.state = 12

                output["xcar"] = 58
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 12:
            if (xobj == 1):
                self.state = 13

                output["xcar"] = 59
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 13:
            if (xobj == 1):
                self.state = 14

                output["xcar"] = 60
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 14:
            if (xobj == 1):
                self.state = 15

                output["xcar"] = 61
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 15:
            if (xobj == 1):
                self.state = 16

                output["xcar"] = 62
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 16:
            if (xobj == 1):
                self.state = 17

                output["xcar"] = 63
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 17:
            if (xobj == 1):
                self.state = 18

                output["xcar"] = 64
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 18:
            if (xobj == 1):
                self.state = 19

                output["xcar"] = 65
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 19:
            if (xobj == 1):
                self.state = 19

                output["xcar"] = 65
                output["vcar"] = 1
            else:
                self._error(xobj)
        elif self.state == 20:
            if (xobj == 1):
                self.state = 0

                output["xcar"] = 1
                output["vcar"] = 10
            else:
                self._error(xobj)
        else:
            raise Exception("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, xobj):
        raise ValueError("Unrecognized input: " + (
            "xobj = {xobj}; ").format(
                xobj=xobj))

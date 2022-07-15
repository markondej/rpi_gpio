#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "../rpi_gpio.hpp"

static PyObject *GPIOError;

static GPIO::Mode GetMode(int mode) {
    switch (mode) {
    case 1:
        return GPIO::Mode::In;
    default:
    }
    return GPIO::Mode::Out;
}

static GPIO::Resistor GetResistor(int resistor) {
    switch (resistor) {
    case 2:
        return GPIO::Resistor::PullUp;
    case 1:
        return GPIO::Resistor::PullDown;
    default:
    }
    return GPIO::Resistor::Off;
}


static PyObject *GPIO_set_mode(PyObject *self, PyObject *args) {
    int pin, mode;
    if (!PyArg_ParseTuple(args, "ii", &pin, &mode)) {
        return NULL;
    }

    GPIO::Controller::GetInstance().SetMode(pin, GetMode(mode));

    Py_RETURN_NONE;
}

static PyObject *GPIO_set_resistor(PyObject *self, PyObject *args) {
    int pin, resistor;
    if (!PyArg_ParseTuple(args, "ii", &pin, &resistor)) {
        return NULL;
    }

    GPIO::Controller::GetInstance().SetResistor(pin, GetResistor(resistor));

    Py_RETURN_NONE;
}

static PyObject *GPIO_set(PyObject *self, PyObject *args) {
    int pin, active;
    if (!PyArg_ParseTuple(args, "ii", &pin, &active)) {
        return NULL;
    }

    GPIO::Controller::GetInstance().Set(pin, active);

    Py_RETURN_NONE;
}

static PyObject *GPIO_set(PyObject *self, PyObject *args) {
    int pin;
    if (!PyArg_ParseTuple(args, "i", &pin)) {
        return NULL;
    }

    return Py_BuildValue("i", GPIO::Controller::GetInstance().Get(pin) ? 1 : 0);
}

static PyMethodDef GPIOMethods[] = {
    { "set_mode",  GPIO_set_mode, METH_VARARGS, "Set GPIO mode" },
    { "set_resistor",  GPIO_set_mode, METH_VARARGS, "Set GPIO pull-up/down resistor configuration" },
    { "set",  GPIO_set, METH_VARARGS, "Set GPIO state" },
    { "get",  GPIO_get, METH_VARARGS, "Get GPIO state" },
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef GPIOModule = {
    PyModuleDef_HEAD_INIT,
    "gpio",
    "Raspberry Pi GPIO module",
    -1,
    GPIOMethods
};

PyMODINIT_FUNC PyInit_gpio(void) {
    GPIO::Controller::GetInstance();
    PyObject *mod = PyModule_Create(&GPIOModule);
    if (mod != NULL) {
        GPIOError = PyErr_NewException("gpio.error", NULL, NULL);
        Py_XINCREF(GPIOError);
        if (PyModule_AddObject(mod, "error", GPIOError) < 0) {
            Py_XDECREF(GPIOError);
            Py_CLEAR(GPIOError);
            Py_DECREF(mod);
            return NULL;
        }
    }
    return mod;
}

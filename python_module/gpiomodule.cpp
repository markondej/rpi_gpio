#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "../rpi_gpio.hpp"

static PyObject *GPIOError;

static GPIO::Mode GetMode(int mode) {
    switch (mode) {
    case 1:
        return GPIO::Mode::In;
    default:
        break;
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
        break;
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

static PyObject *GPIO_get(PyObject *self, PyObject *args) {
    int pin;
    if (!PyArg_ParseTuple(args, "i", &pin)) {
        return NULL;
    }

    return Py_BuildValue("i", GPIO::Controller::GetInstance().Get(pin) ? 1 : 0);
}

static PyObject *GPIO_set_schedule(PyObject *self, PyObject *args) {
    int interval;
    PyObject *schedule;
    if (!PyArg_ParseTuple(args, "Oi", &schedule, &interval)) {
        return NULL;
    }

    std::vector<GPIO::Event> events;
    try {
        PyObject *eventIter = PyObject_GetIter(schedule);
        if (eventIter) {
            while (true) {
                PyObject *element = PyIter_Next(eventIter);
                if (!element) {
                    break;
                }
                PyObject *propIter = PyObject_GetIter(element);
                if (!propIter) {
                    throw std::exception();
                }
                std::size_t propNum = 0;
                GPIO::Event current = { 0, 0, false };
                while (true) {
                    PyObject *prop = PyIter_Next(propIter);
                    if (!propIter) {
                        if (!propNum) {
                            break;
                        }
                        throw std::exception();
                    }
                    if (!PyLong_Check(prop)) {
                        throw std::exception();
                    }
                    switch (propNum) {
                    case 2:
                        current.high = PyLong_AsLong(prop);
                        events.push_back(current);
                        break;
                    case 1:
                        current.number = PyLong_AsLong(prop);
                        break;
                    default:
                        current.time = PyLong_AsLongLong(prop);
                        break;
                    }
                    propNum = (propNum < 2) ? propNum + 1 : 0;
                }
            }
        }
    } catch (...) {
        PyErr_SetString(GPIOError, "Wrong schedule format");
        return NULL;
    }

    GPIO::Controller::GetInstance().SetSchedule(events, interval);

    Py_RETURN_NONE;
}

static PyObject *GPIO_get_events(PyObject *self, PyObject *args) {
    if (!PyArg_ParseTuple(args, "")) {
        return NULL;
    }

    auto events = GPIO::Controller::GetInstance().GetEvents();

    PyObject* list = PyList_New(events.size());
    for (std::size_t i = 0; i < events.size(); ++i) {
        PyObject* element = PyList_New(3);
        PyObject* timestamp = Py_BuildValue("i", events[i].time);
        PyObject* number = Py_BuildValue("i", events[i].number);
        PyObject* high = Py_BuildValue("i", events[i].high ? 1 : 0);
        PyList_SetItem(element, 0, timestamp);
        PyList_SetItem(element, 1, number);
        PyList_SetItem(element, 2, high);
        PyList_SetItem(list, i, element);
    }

    return list;
}

static PyMethodDef GPIOMethods[] = {
    { "set_mode",  GPIO_set_mode, METH_VARARGS, "Sets GPIO mode" },
    { "set_resistor",  GPIO_set_resistor, METH_VARARGS, "Sets GPIO pull-up/down resistor configuration" },
    { "set",  GPIO_set, METH_VARARGS, "Sets GPIO state" },
    { "get",  GPIO_get, METH_VARARGS, "Returns GPIO state" },
    { "get_events",  GPIO_get_events, METH_VARARGS, "Retruns registered GPIO state" },
    { "set_schedule",  GPIO_set_schedule, METH_VARARGS, "Sets GPIO event schedule" },
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

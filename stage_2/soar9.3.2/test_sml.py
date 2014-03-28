#! /usr/bin/env python
import sys
sys.path.append('/home/sampfeiffer/soar/bin')
import Python_sml_ClientInterface as sml

k = sml.Kernel.CreateKernelInNewThread()
a = k.CreateAgent('soar')
print a.ExecuteCommandLine('echo hello world')
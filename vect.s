.extern undefinedStub
.extern _start
.extern svcStub
.extern pabtStub
.extern dabtStub
.extern irqStub
.extern fiqStub

.globl exception_vectors
.section .exception_vectors, "a", %progbits
b.w _start//0
b.w undefinedStub//4
b.w svcStub//8
b.w pabtStub//12
b.w dabtStub//16
b.w _start// unknown/reserved
b.w irqStub//24
b.w fiqStub//28

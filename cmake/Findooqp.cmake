# - Try to find ooqp
# Once done tihs will define
#   OOQP_FOUND
#   OOQP_INCLUDE_DIRS
#   OOQP_LIBRARIES
#   OOQP_DEFINITIONS

find_path(OOQP_INCLUDE_DIR NAMES
  ooqp/cBounds.h
  ooqp/DeSymIndefSolver.h
  ooqp/MehrotraSolver.h
  ooqp/QpBoundVars.h
  ooqp/SmartPointer.h
  ooqp/cMpsReader.h
  ooqp/DeSymPSDSolver.h
  ooqp/MpsReader.h
  ooqp/QpGenData.h
  ooqp/Solver.h
  ooqp/cMpsReaderPetsc.h
  ooqp/DoubleLinearSolver.h
  ooqp/OoqpBlas.h
  ooqp/QpGenDense.h
  ooqp/SparseGenMatrix.h
  ooqp/cQpBoundDense.h
  ooqp/DoubleMatrix.h
  ooqp/OoqpMonitorData.h
  ooqp/QpGenDenseLinsys.h
  ooqp/SparseGenMatrixHandle.h
  ooqp/cQpBound.h
  ooqp/DoubleMatrixHandle.h
  ooqp/OoqpMonitor.h
  ooqp/QpGenDriver.h
  ooqp/SparseLinearAlgebraPackage.h
  ooqp/cQpBoundPetsc.h
  ooqp/DoubleMatrixTypes.h
  ooqp/OoqpPetscMonitor.h
  ooqp/QpGen.h
  ooqp/SparseStorage.h
  ooqp/cQpGenDense.h
  ooqp/GondzioSolver.h
  ooqp/OoqpStartStrategy.h
  ooqp/QpGenLinsys.h
  ooqp/SparseStorageHandle.h
  ooqp/cQpGen.h
  ooqp/hash.h
  ooqp/OoqpVector.h
  ooqp/QpGenResiduals.h
  ooqp/SparseSymMatrix.h
  ooqp/cQpGenPetsc.h
  ooqp/HuberData.h
  ooqp/OoqpVectorHandle.h
  ooqp/QpGenSparseLinsys.h
  ooqp/SparseSymMatrixHandle.h
  ooqp/cQpGenSparse.h
  ooqp/Huber.h
  ooqp/OoqpVersion.h
  ooqp/QpGenSparseMa27.h
  ooqp/Status.h
  ooqp/Data.h
  ooqp/HuberLinsys.h
  ooqp/ProblemFormulation.h
  ooqp/QpGenSparseMa57.h
  ooqp/SvmData.h
  ooqp/DenseGenMatrix.h
  ooqp/HuberResiduals.h
  ooqp/QpBoundData.h
  ooqp/QpGenSparseOblio.h
  ooqp/Svm.h
  ooqp/DenseGenMatrixHandle.h
  ooqp/HuberVars.h
  ooqp/QpBoundDense.h
  ooqp/QpGenSparseSeq.h
  ooqp/SvmLinsys.h
  ooqp/DenseLinearAlgebraPackage.h
  ooqp/IotrRefCount.h
  ooqp/QpBoundDenseLinsys.h
  ooqp/QpGenSparseSuperLu.h
  ooqp/SvmResiduals.h
  ooqp/DenseStorage.h
  ooqp/LinearAlgebraPackage.h
  ooqp/QpBound.h
  ooqp/QpGenVars.h
  ooqp/SvmVars.h
  ooqp/DenseStorageHandle.h
  ooqp/LinearSystem.h
  ooqp/QpBoundLinsys.h
  ooqp/Residuals.h
  ooqp/Variables.h
  ooqp/DenseSymMatrix.h
  ooqp/Ma27Solver.h
  ooqp/QpBoundPetsc.h
  ooqp/SimpleVector.h
  ooqp/VectorUtilities.h
  ooqp/DenseSymMatrixHandle.h
  ooqp/Ma57Solver.h
  ooqp/QpBoundResiduals.h
  ooqp/SimpleVectorHandle.h
  HINTS /usr/local/include
)

set(LIB_DIR /usr/local/lib)

find_library(OOQP_BASE NAMES ooqpbase HINTS ${LIB_DIR})
find_library(OOQP_DENSE NAMES ooqpdense HINTS ${LIB_DIR})
find_library(OOQP_GENSPARSE NAMES ooqpgensparse HINTS ${LIB_DIR})
find_library(OOQP_MEHROTRA NAMES ooqpmehrotra HINTS ${LIB_DIR})
find_library(OOQP_BOUND NAMES ooqpbound HINTS ${LIB_DIR})
find_library(OOQP_GENDENSE NAMES ooqpgendense HINTS ${LIB_DIR})
find_library(OOQP_GONDZIO NAMES ooqpgondzio HINTS ${LIB_DIR})
find_library(OOQP_SPARSE NAMES ooqpsparse HINTS ${LIB_DIR})

set(OOQP_LIBRARY ${OOQP_BASE} ${OOQP_DENSE} ${OOQP_GENSPARSE} ${OOQP_MEHROTRA} ${OOQP_BOUND} ${OOQP_GENDENSE} ${OOQP_GONDZIO} ${OOQP_SPARSE}
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OOQP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ooqp  DEFAULT_MSG
                                  OOQP_LIBRARY OOQP_INCLUDE_DIR)

mark_as_advanced(OOQP_INCLUDE_DIR OOQP_LIBRARY)
set(OOQP_LIBRARIES ${OOQP_LIBRARY} )
set(OOQP_INCLUDE_DIRS ${OOQP_INCLUDE_DIR} )
MESSAGE( STATUS "OOQP_INCLUDE_DIRS: " ${OOQP_INCLUDE_DIRS} )
MESSAGE( STATUS "OOQP_LIBRARIES: " ${OOQP_LIBRARIES} )

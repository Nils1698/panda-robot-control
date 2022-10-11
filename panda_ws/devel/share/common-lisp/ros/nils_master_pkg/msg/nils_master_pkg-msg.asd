
(cl:in-package :asdf)

(defsystem "nils_master_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PredictedPoses" :depends-on ("_package_PredictedPoses"))
    (:file "_package_PredictedPoses" :depends-on ("_package"))
  ))
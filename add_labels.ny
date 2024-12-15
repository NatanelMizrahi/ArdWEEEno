; A Nyquist script for Audacity used to add labels for each track in the project matching the track name

(dolist (track (aud-get-info "tracks"))
  (let* ((track-name (second (assoc 'name track)))
         (start-time (second (assoc 'start track)))
         (end-time (second (assoc 'end track)))))
  (aud-do "AddLabel:")
  (aud-do (format nil "SetLabel: Label=0 Start=~s End=~s Text=~s" 
    (second (assoc 'start track)) 
    (second (assoc 'end track)) 
    (second (assoc 'name track)))))
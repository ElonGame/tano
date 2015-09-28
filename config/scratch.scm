(define (make-intro)
 (let ((move-speed 15.0))
   (define (explode-time) 22.0)
   (define (funky value) (+ value 1))
   (lambda args
     (apply
      (case (car args)
        ((explode-time) explode-time)
        ((move-speed) move-speed)
        ((funky) funky))
      (cdr args)))))

(define intro (make-intro))
